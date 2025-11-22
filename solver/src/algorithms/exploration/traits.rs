use crate::{
    maze::{UnboundedMaze, UnboundedPosition},
    ros::types::{MoveDirection, SensorsStates},
};

pub trait ExplorationAlgorithm {
    /// returns None when target is reached
    fn next_move(
        &mut self,
        current_pos: UnboundedPosition,
        sensors: &SensorsStates,
        maze: &UnboundedMaze,
        target_found: bool,
        target_pos: Option<UnboundedPosition>,
    ) -> eyre::Result<Option<MoveDirection>>;

    #[allow(dead_code)]
    fn name(&self) -> &'static str;

    fn reset(&mut self);
}
