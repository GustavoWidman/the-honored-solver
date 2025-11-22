use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::{
    maze::{Cell, UnboundedMaze, UnboundedPosition},
    ros::ROSInterface,
};

use crate::algorithms::{exploration::ExplorationAlgorithm, pathfinding::PathResult};

/// explores maze using only sensor data with unbounded coordinates
pub struct BlindSolver<E: ExplorationAlgorithm> {
    algorithm: E,
    delay: Duration,
}

impl<E: ExplorationAlgorithm> BlindSolver<E> {
    pub fn new(algorithm: E, delay_ms: u64) -> Self {
        Self {
            algorithm,
            delay: Duration::from_millis(delay_ms),
        }
    }

    pub async fn solve(&mut self, ros: Arc<ROSInterface>) -> eyre::Result<PathResult> {
        log::debug!("starting blind exploration");
        log::debug!("using unbounded coordinate system");

        self.algorithm.reset();
        let mut maze = UnboundedMaze::new();
        let mut sensor_rx = ros.subscribe_sensors();

        log::debug!("waiting for sensors");
        let initial_sensors = sensor_rx.recv().await?;

        let mut current_pos = UnboundedPosition::new(0, 0);
        maze.set(current_pos, Cell::Robot);
        maze.update_from_sensors(current_pos, &initial_sensors);

        log::info!("starting at origin");

        let mut target_found = false;
        let mut target_pos: Option<UnboundedPosition> = None;
        let mut steps = 0;

        let total_start = Instant::now();
        let planning_start = Instant::now();

        loop {
            let sensors = sensor_rx.recv().await?;
            maze.update_from_sensors(current_pos, &sensors);

            if !target_found {
                target_pos = Self::detect_target_in_sensors(current_pos, &sensors);
                if target_pos.is_some() {
                    target_found = true;
                    let pos = target_pos.unwrap();
                    log::info!("target spotted at ({}, {})", pos.row, pos.col);
                }
            }

            let next_move =
                self.algorithm
                    .next_move(current_pos, &sensors, &maze, target_found, target_pos)?;

            if next_move.is_none() {
                log::info!("reached target");
                break;
            }

            let direction = next_move.unwrap();

            if self.delay.as_millis() > 0 {
                tokio::time::sleep(self.delay).await;
            }

            log::debug!(
                "step {}: {:?} from ({}, {})",
                steps + 1,
                direction,
                current_pos.row,
                current_pos.col
            );

            let response = ros.move_cmd(direction).await?;
            if !response.success {
                eyre::bail!("move failed at step {}: {:?}", steps + 1, direction);
            }

            maze.set(current_pos, Cell::Free);
            current_pos = current_pos.move_in_direction(direction);
            maze.set(current_pos, Cell::Robot);

            steps += 1;

            if steps > 10_000 {
                eyre::bail!("too many steps ({}) - possible infinite loop", steps);
            }
        }

        let planning_time = planning_start.elapsed();
        let execution_time = total_start.elapsed();

        Ok(PathResult::new(steps, planning_time, execution_time))
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
