use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::{
    maze::{BoundedMaze, Cell, Position, UnboundedMaze, UnboundedPosition},
    ros::ROSInterface,
};

use crate::algorithms::{
    exploration::ExplorationAlgorithm, pathfinding::PathResult, pathfinding::PathfindingAlgorithm,
};
use crate::ros::types::{MoveDirection, SensorsStates};

/// explores maze using only sensor data with unbounded coordinates
pub struct BlindSolver<E: ExplorationAlgorithm, P: PathfindingAlgorithm> {
    exploration: E,
    pathfinding: P,
    delay: Duration,
}

impl<E: ExplorationAlgorithm, P: PathfindingAlgorithm> BlindSolver<E, P> {
    pub fn new(exploration: E, pathfinding: P, delay_ms: u64) -> Self {
        Self {
            exploration,
            pathfinding,
            delay: Duration::from_millis(delay_ms),
        }
    }

    pub async fn solve(&mut self, ros: Arc<ROSInterface>) -> eyre::Result<PathResult> {
        log::debug!("starting blind exploration");
        log::debug!("using unbounded coordinate system");

        self.exploration.reset();
        let mut maze = UnboundedMaze::new();
        let mut sensor_rx = ros.subscribe_sensors();

        log::debug!("waiting for sensors");
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // drain any stale sensor messages from before reset
        while !sensor_rx.is_empty() {
            let _ = sensor_rx.recv().await;
        }

        let initial_sensors = sensor_rx.recv().await?;

        let mut current_pos = UnboundedPosition::new(0, 0);
        maze.set(current_pos, Cell::Robot);
        maze.update_from_sensors(current_pos, &initial_sensors);

        log::info!("starting at origin");

        let mut sensor_cache: HashMap<UnboundedPosition, SensorsStates> = HashMap::new();
        sensor_cache.insert(current_pos, initial_sensors.clone());

        let total_start = Instant::now();
        let mut total_planning_time = Duration::default();

        log::info!("phase 1: exploring maze with {}", self.exploration.name());

        let (target_position, exploration_steps) = self
            .explore_phase(
                &ros,
                &mut maze,
                &mut sensor_rx,
                &mut sensor_cache,
                &mut current_pos,
                &mut total_planning_time,
            )
            .await?;

        log::info!(
            "exploration complete: found target at ({}, {}) in {} steps",
            target_position.row,
            target_position.col,
            exploration_steps
        );

        log::info!(
            "phase 2: planning optimal path with {}",
            self.pathfinding.name()
        );

        let planning_start = Instant::now();
        let (bounded_maze, start, target) = self.convert_to_bounded(&maze, target_position)?;
        let optimal_path = self
            .pathfinding
            .find_path(&bounded_maze, start, target)
            .ok_or_else(|| eyre::eyre!("no path found to target"))?;
        total_planning_time += planning_start.elapsed();

        log::info!("planned optimal path: {} steps", optimal_path.len());

        log::info!("resetting maze and executing optimal path");

        ros.reset(false, String::new()).await?;
        tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

        let execution_steps = self.execute_path(&ros, &optimal_path).await?;

        let total_time = total_start.elapsed();
        let execution_time = total_time - total_planning_time;

        log::info!(
            "total: {} exploration + {} execution = {} steps",
            exploration_steps,
            execution_steps,
            exploration_steps + execution_steps
        );

        Ok(PathResult::new(
            exploration_steps + execution_steps,
            total_planning_time,
            execution_time,
        ))
    }

    async fn explore_phase(
        &mut self,
        ros: &Arc<ROSInterface>,
        maze: &mut UnboundedMaze,
        sensor_rx: &mut tokio::sync::broadcast::Receiver<SensorsStates>,
        sensor_cache: &mut HashMap<UnboundedPosition, SensorsStates>,
        current_pos: &mut UnboundedPosition,
        total_planning_time: &mut Duration,
    ) -> eyre::Result<(UnboundedPosition, usize)> {
        let mut target_pos: Option<UnboundedPosition> = None;
        let mut steps = 0;

        loop {
            let sensors = if let Some(cached_sensors) = sensor_cache.get(current_pos) {
                log::debug!(
                    "cache hit! using cached sensors for ({}, {})",
                    current_pos.row,
                    current_pos.col
                );
                cached_sensors.clone()
            } else {
                // drain sensors
                while !sensor_rx.is_empty() {
                    let _ = sensor_rx.recv().await;
                }

                let fresh_sensors = sensor_rx.recv().await?;
                log::trace!(
                    "fresh sensors for ({}, {})",
                    current_pos.row,
                    current_pos.col
                );
                sensor_cache.insert(*current_pos, fresh_sensors.clone());
                fresh_sensors
            };
            maze.update_from_sensors(*current_pos, &sensors);

            // detect target but don't stop exploring
            if target_pos.is_none() {
                if let Some(pos) = Self::detect_target_in_sensors(*current_pos, &sensors) {
                    log::info!("target spotted at ({}, {})", pos.row, pos.col);
                    target_pos = Some(pos);
                }
            }

            let planning_start = Instant::now();
            let next_move = self.exploration.next_move(*current_pos, &sensors, maze)?;
            *total_planning_time += planning_start.elapsed();

            if next_move.is_none() {
                log::info!("exploration complete after {} steps", steps);
                break;
            }

            let direction = next_move.unwrap();

            if self.delay.as_millis() > 0 {
                tokio::time::sleep(self.delay).await;
            }

            log::debug!(
                "exploration step {}: {:?} from ({}, {})",
                steps + 1,
                direction,
                current_pos.row,
                current_pos.col
            );

            let response = ros.move_cmd(direction).await?;
            if !response.success {
                eyre::bail!("move failed at step {}: {:?}", steps + 1, direction);
            }

            maze.set(*current_pos, Cell::Free);
            *current_pos = current_pos.move_in_direction(direction);
            maze.set(*current_pos, Cell::Robot);

            steps += 1;

            if steps > 10_000 {
                eyre::bail!("too many steps ({}) - possible infinite loop", steps);
            }
        }

        target_pos
            .ok_or_else(|| eyre::eyre!("exploration complete but target never spotted"))
            .map(|pos| (pos, steps))
    }

    fn convert_to_bounded(
        &self,
        unbounded: &UnboundedMaze,
        target: UnboundedPosition,
    ) -> eyre::Result<(BoundedMaze, Position, Position)> {
        let (min_row, max_row, min_col, max_col) = unbounded
            .get_bounds()
            .ok_or_else(|| eyre::eyre!("empty maze"))?;

        let height = (max_row - min_row + 1) as usize;
        let width = (max_col - min_col + 1) as usize;

        let mut grid_data = vec!["u".to_string(); height * width];

        for row in min_row..=max_row {
            for col in min_col..=max_col {
                let pos = UnboundedPosition::new(row, col);
                let cell = unbounded.get(pos);
                let grid_row = (row - min_row) as usize;
                let grid_col = (col - min_col) as usize;
                let idx = grid_row * width + grid_col;

                grid_data[idx] = match cell {
                    Cell::Free => "f".to_string(),
                    Cell::Blocked => "b".to_string(),
                    Cell::Target => "t".to_string(),
                    Cell::Robot => "r".to_string(),
                    Cell::Unknown => "u".to_string(),
                };
            }
        }

        let bounded = BoundedMaze::from_flattened(grid_data, vec![height as u8, width as u8])?;

        let start = Position::new((0 - min_row) as usize, (0 - min_col) as usize);
        let target_pos = Position::new(
            (target.row - min_row) as usize,
            (target.col - min_col) as usize,
        );

        Ok((bounded, start, target_pos))
    }

    async fn execute_path(
        &self,
        ros: &Arc<ROSInterface>,
        path: &[MoveDirection],
    ) -> eyre::Result<usize> {
        let mut sensor_rx = ros.subscribe_sensors();

        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // drain sensors
        while !sensor_rx.is_empty() {
            let _ = sensor_rx.recv().await;
        }

        sensor_rx.recv().await?;

        for (i, direction) in path.iter().enumerate() {
            if self.delay.as_millis() > 0 {
                tokio::time::sleep(self.delay).await;
            }

            log::debug!("executing step {}/{}: {:?}", i + 1, path.len(), direction);

            let response = ros.move_cmd(*direction).await?;
            if !response.success {
                eyre::bail!("execution failed at step {}: {:?}", i + 1, direction);
            }
        }

        log::info!("reached target");
        Ok(path.len())
    }

    fn detect_target_in_sensors(
        current: UnboundedPosition,
        sensors: &crate::ros::types::SensorsStates,
    ) -> Option<UnboundedPosition> {
        use crate::ros::types::SensorState;

        let directions = [
            (&sensors.up, -1isize, 0isize),
            (&sensors.down, 1, 0),
            (&sensors.left, 0, -1),
            (&sensors.right, 0, 1),
            (&sensors.up_left, -1, -1),
            (&sensors.up_right, -1, 1),
            (&sensors.down_left, 1, -1),
            (&sensors.down_right, 1, 1),
        ];

        for (state, dr, dc) in directions {
            if matches!(state, SensorState::Target) {
                let new_row = current.row + dr;
                let new_col = current.col + dc;
                return Some(UnboundedPosition::new(new_row, new_col));
            }
        }

        None
    }
}
