use std::time::Duration;

use crate::{
    maze::{BoundedMaze, Position},
    ros::types::MoveDirection,
};

pub trait PathfindingAlgorithm {
    fn find_path(
        &self,
        maze: &BoundedMaze,
        start: Position,
        target: Position,
    ) -> Option<Vec<MoveDirection>>;

    #[allow(dead_code)]
    fn name(&self) -> &'static str;
}

pub struct PathResult {
    pub steps: usize,
    pub planning_time: Duration,
    pub execution_time: Duration,
    pub total_time: Duration,
}

impl PathResult {
    pub fn new(steps: usize, planning_time: Duration, execution_time: Duration) -> Self {
        let total_time = planning_time + execution_time;
        Self {
            steps,
            planning_time,
            execution_time,
            total_time,
        }
    }
}
