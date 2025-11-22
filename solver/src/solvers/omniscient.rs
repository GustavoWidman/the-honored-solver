use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::{maze::BoundedMaze, ros::ROSInterface};

use crate::algorithms::pathfinding::{PathResult, PathfindingAlgorithm};

pub struct OmniscientSolver<A: PathfindingAlgorithm> {
    algorithm: A,
    delay: Duration,
}

impl<A: PathfindingAlgorithm> OmniscientSolver<A> {
    pub fn new(algorithm: A, delay_ms: u64) -> Self {
        Self {
            algorithm,
            delay: Duration::from_millis(delay_ms),
        }
    }

    pub async fn solve(&self, ros: Arc<ROSInterface>) -> eyre::Result<PathResult> {
        log::debug!("fetching maze map");
        let map_response = ros.get_map().await?;

        let maze = BoundedMaze::from_flattened(
            map_response.occupancy_grid_flattened,
            map_response.occupancy_grid_shape,
        )?;

        let start = maze
            .find_robot()
            .ok_or_else(|| eyre::eyre!("robot not found in maze"))?;
        let target = maze
            .find_target()
            .ok_or_else(|| eyre::eyre!("target not found in maze"))?;

        log::debug!(
            "{}x{} maze: ({}, {}) → ({}, {})",
            maze.height(),
            maze.width(),
            start.row,
            start.col,
            target.row,
            target.col
        );

        let planning_start = Instant::now();
        let path = self
            .algorithm
            .find_path(&maze, start, target)
            .ok_or_else(|| eyre::eyre!("no path found"))?;
        let planning_time = planning_start.elapsed();

        log::info!("planned {} steps in {:?}", path.len(), planning_time);

        log::debug!("executing");
        let execution_start = Instant::now();

        for (step, &direction) in path.iter().enumerate() {
            if self.delay.as_millis() > 0 {
                tokio::time::sleep(self.delay).await;
            }

            log::debug!("step {}/{}: {:?}", step + 1, path.len(), direction);
            let response = ros.move_cmd(direction).await?;

            if !response.success {
                eyre::bail!("move failed at step {}: {:?}", step + 1, direction);
            }
        }

        let execution_time = execution_start.elapsed();

        log::info!("reached target");

        Ok(PathResult::new(path.len(), planning_time, execution_time))
    }
}
