use std::collections::HashSet;

use crate::{
    maze::{UnboundedMaze, UnboundedPosition},
    ros::types::{MoveDirection, SensorsStates},
};

use super::traits::ExplorationAlgorithm;

/// wall follower using left-hand rule
pub struct WallFollower {
    facing: MoveDirection,
    first_move: bool,
    visited: HashSet<UnboundedPosition>,
    start_pos: Option<UnboundedPosition>,
    returned_to_start: bool,
}

impl WallFollower {
    pub fn new() -> Self {
        Self {
            facing: MoveDirection::Up,
            first_move: true,
            visited: HashSet::new(),
            start_pos: None,
            returned_to_start: false,
        }
    }

    fn turn_left(&self) -> MoveDirection {
        match self.facing {
            MoveDirection::Up => MoveDirection::Left,
            MoveDirection::Left => MoveDirection::Down,
            MoveDirection::Down => MoveDirection::Right,
            MoveDirection::Right => MoveDirection::Up,
        }
    }

    fn turn_right(&self) -> MoveDirection {
        match self.facing {
            MoveDirection::Up => MoveDirection::Right,
            MoveDirection::Right => MoveDirection::Down,
            MoveDirection::Down => MoveDirection::Left,
            MoveDirection::Left => MoveDirection::Up,
        }
    }

    fn turn_around(&self) -> MoveDirection {
        match self.facing {
            MoveDirection::Up => MoveDirection::Down,
            MoveDirection::Down => MoveDirection::Up,
            MoveDirection::Left => MoveDirection::Right,
            MoveDirection::Right => MoveDirection::Left,
        }
    }

    fn can_move(&self, direction: MoveDirection, sensors: &SensorsStates) -> bool {
        use crate::ros::types::SensorState;

        let sensor_state = match direction {
            MoveDirection::Up => &sensors.up,
            MoveDirection::Down => &sensors.down,
            MoveDirection::Left => &sensors.left,
            MoveDirection::Right => &sensors.right,
        };

        // treat target as blocked during exploration - we don't want to reach it yet
        matches!(sensor_state, SensorState::Free)
    }
}

impl ExplorationAlgorithm for WallFollower {
    fn next_move(
        &mut self,
        current_pos: UnboundedPosition,
        sensors: &SensorsStates,
        _maze: &UnboundedMaze,
    ) -> eyre::Result<Option<MoveDirection>> {
        // track starting position
        if self.start_pos.is_none() {
            self.start_pos = Some(current_pos);
        }

        // mark current position as visited
        self.visited.insert(current_pos);

        // if we've returned to start after visiting other positions, exploration is complete
        if self.returned_to_start {
            return Ok(None);
        }

        if let Some(start) = self.start_pos
            && current_pos == start
            && self.visited.len() > 1
        {
            self.returned_to_start = true;
            log::debug!(
                "wall follower returned to start after visiting {} positions",
                self.visited.len()
            );
        }

        if self.first_move {
            self.first_move = false;
            for dir in [
                MoveDirection::Up,
                MoveDirection::Right,
                MoveDirection::Down,
                MoveDirection::Left,
            ] {
                if self.can_move(dir, sensors) {
                    self.facing = dir;
                    return Ok(Some(dir));
                }
            }
            eyre::bail!("No valid initial move - completely surrounded!");
        }

        let left = self.turn_left();
        let straight = self.facing;
        let right = self.turn_right();
        let back = self.turn_around();

        let next_dir = if self.can_move(left, sensors) {
            left
        } else if self.can_move(straight, sensors) {
            straight
        } else if self.can_move(right, sensors) {
            right
        } else if self.can_move(back, sensors) {
            back
        } else {
            eyre::bail!("Completely blocked - no valid moves!");
        };

        self.facing = next_dir;
        Ok(Some(next_dir))
    }

    fn name(&self) -> &'static str {
        "Wall Follower (Left-Hand Rule)"
    }

    fn reset(&mut self) {
        self.facing = MoveDirection::Up;
        self.first_move = true;
        self.visited.clear();
        self.start_pos = None;
        self.returned_to_start = false;
    }
}
