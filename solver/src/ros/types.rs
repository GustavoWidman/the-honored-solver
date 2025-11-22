use r2r::cg_interfaces::msg::RobotSensors;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveDirection {
    Up,
    Down,
    Left,
    Right,
}

impl MoveDirection {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Up => "up",
            Self::Down => "down",
            Self::Left => "left",
            Self::Right => "right",
        }
    }
}

impl From<&str> for MoveDirection {
    fn from(s: &str) -> Self {
        match s {
            "up" => Self::Up,
            "down" => Self::Down,
            "left" => Self::Left,
            "right" => Self::Right,
            _ => panic!("Invalid move direction: {}", s),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SensorState {
    Blocked,
    Target,
    Free,
}

impl From<&str> for SensorState {
    fn from(s: &str) -> Self {
        match s.to_lowercase().as_str() {
            "b" => Self::Blocked,
            "t" => Self::Target,
            "f" => Self::Free,
            _ => panic!("Invalid sensor state: {}", s),
        }
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct SensorsStates {
    pub up: SensorState,
    pub down: SensorState,
    pub left: SensorState,
    pub right: SensorState,
    pub up_left: SensorState,
    pub up_right: SensorState,
    pub down_left: SensorState,
    pub down_right: SensorState,
}

impl From<RobotSensors> for SensorsStates {
    fn from(sensors: RobotSensors) -> Self {
        Self {
            up: SensorState::from(sensors.up.as_str()),
            down: SensorState::from(sensors.down.as_str()),
            left: SensorState::from(sensors.left.as_str()),
            right: SensorState::from(sensors.right.as_str()),
            up_left: SensorState::from(sensors.up_left.as_str()),
            up_right: SensorState::from(sensors.up_right.as_str()),
            down_left: SensorState::from(sensors.down_left.as_str()),
            down_right: SensorState::from(sensors.down_right.as_str()),
        }
    }
}
