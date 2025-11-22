use crate::{
    maze::{UnboundedMaze, UnboundedPosition},
    ros::types::{MoveDirection, SensorsStates},
};

use super::traits::ExplorationAlgorithm;

/// wall follower using left-hand rule
pub struct WallFollower {
    facing: MoveDirection,
    first_move: bool,
}

impl WallFollower {
    pub fn new() -> Self {
        Self {
            facing: MoveDirection::Up,
            first_move: true,
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

        !matches!(sensor_state, SensorState::Blocked)
    }

    /// assumes target is adjacent in cardinal direction
    fn direction_to_target(
        &self,
        current: UnboundedPosition,
        target: UnboundedPosition,
    ) -> eyre::Result<MoveDirection> {
        let dr = target.row - current.row;
        let dc = target.col - current.col;

        match (dr, dc) {
            (-1, 0) => Ok(MoveDirection::Up),
            (1, 0) => Ok(MoveDirection::Down),
            (0, -1) => Ok(MoveDirection::Left),
            (0, 1) => Ok(MoveDirection::Right),
            _ => eyre::bail!("Target is not adjacent in a cardinal direction"),
        }
    }
}

impl ExplorationAlgorithm for WallFollower {
    fn next_move(
        &mut self,
        current_pos: UnboundedPosition,
        sensors: &SensorsStates,
        _maze: &UnboundedMaze,
        target_found: bool,
        target_pos: Option<UnboundedPosition>,
    ) -> eyre::Result<Option<MoveDirection>> {
        if target_found {
            if let Some(target) = target_pos {
                if current_pos == target {
                    return Ok(None);
                }

                if let Ok(direction) = self.direction_to_target(current_pos, target) {
                    if self.can_move(direction, sensors) {
                        self.facing = direction;
                        return Ok(Some(direction));
                    }
                }
            }
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
    }
}
