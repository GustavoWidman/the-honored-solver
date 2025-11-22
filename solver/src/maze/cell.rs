use crate::ros::types::SensorState;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cell {
    Free,
    Blocked,
    Target,
    Robot,
    Unknown,
}

impl Cell {
    pub fn from_str(s: &str) -> Self {
        match s {
            "f" => Self::Free,
            "b" => Self::Blocked,
            "t" => Self::Target,
            "r" => Self::Robot,
            _ => Self::Unknown,
        }
    }

    pub fn is_walkable(self) -> bool {
        matches!(self, Self::Free | Self::Target | Self::Robot)
    }
}

impl From<SensorState> for Cell {
    fn from(state: SensorState) -> Self {
        match state {
            SensorState::Free => Self::Free,
            SensorState::Blocked => Self::Blocked,
            SensorState::Target => Self::Target,
        }
    }
}
