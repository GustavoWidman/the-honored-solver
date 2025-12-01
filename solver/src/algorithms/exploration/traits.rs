use crate::{
    maze::{UnboundedMaze, UnboundedPosition},
    ros::types::{MoveDirection, SensorsStates},
};

pub trait ExplorationAlgorithm {
    /// returns None when exploration is complete
    fn next_move(
        &mut self,
        current_pos: UnboundedPosition,
        sensors: &SensorsStates,
        maze: &UnboundedMaze,
    ) -> eyre::Result<Option<MoveDirection>>;

    fn name(&self) -> &'static str;

    fn reset(&mut self);
}
