use std::collections::HashMap;

use super::cell::Cell;
use crate::ros::types::{MoveDirection, SensorState, SensorsStates};

/// position with signed coordinates for unbounded exploration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct UnboundedPosition {
    pub row: isize,
    pub col: isize,
}

impl UnboundedPosition {
    pub fn new(row: isize, col: isize) -> Self {
        Self { row, col }
    }

    pub fn move_in_direction(&self, direction: MoveDirection) -> UnboundedPosition {
        match direction {
            MoveDirection::Up => Self::new(self.row - 1, self.col),
            MoveDirection::Down => Self::new(self.row + 1, self.col),
            MoveDirection::Left => Self::new(self.row, self.col - 1),
            MoveDirection::Right => Self::new(self.row, self.col + 1),
        }
    }

    pub fn neighbors(&self) -> impl Iterator<Item = (UnboundedPosition, MoveDirection)> {
        [
            (Self::new(self.row - 1, self.col), MoveDirection::Up),
            (Self::new(self.row + 1, self.col), MoveDirection::Down),
            (Self::new(self.row, self.col - 1), MoveDirection::Left),
            (Self::new(self.row, self.col + 1), MoveDirection::Right),
        ]
        .into_iter()
    }
}

/// unbounded maze that grows dynamically using hashmap for sparse storage
pub struct UnboundedMaze {
    cells: HashMap<UnboundedPosition, Cell>,
}

impl UnboundedMaze {
    pub fn new() -> Self {
        Self {
            cells: HashMap::new(),
        }
    }

    /// returns Unknown if not yet explored
    pub fn get(&self, pos: UnboundedPosition) -> Cell {
        self.cells.get(&pos).copied().unwrap_or(Cell::Unknown)
    }

    pub fn set(&mut self, pos: UnboundedPosition, cell: Cell) {
        self.cells.insert(pos, cell);
    }

    pub fn is_walkable(&self, pos: UnboundedPosition) -> bool {
        match self.get(pos) {
            Cell::Free | Cell::Target | Cell::Robot => true,
            _ => false,
        }
    }

    pub fn neighbors(&self, pos: UnboundedPosition) -> Vec<(UnboundedPosition, MoveDirection)> {
        pos.neighbors()
            .filter(|(neighbor_pos, _)| self.is_walkable(*neighbor_pos))
            .collect()
    }

    pub fn get_bounds(&self) -> Option<(isize, isize, isize, isize)> {
        if self.cells.is_empty() {
            return None;
        }

        let mut min_row = isize::MAX;
        let mut max_row = isize::MIN;
        let mut min_col = isize::MAX;
        let mut max_col = isize::MIN;

        for pos in self.cells.keys() {
            min_row = min_row.min(pos.row);
            max_row = max_row.max(pos.row);
            min_col = min_col.min(pos.col);
            max_col = max_col.max(pos.col);
        }

        Some((min_row, max_row, min_col, max_col))
    }

    pub fn update_from_sensors(&mut self, pos: UnboundedPosition, sensors: &SensorsStates) {
        self.set(pos, Cell::Robot);
        let sensor_data = [
            (pos.row - 1, pos.col, &sensors.up),
            (pos.row + 1, pos.col, &sensors.down),
            (pos.row, pos.col - 1, &sensors.left),
            (pos.row, pos.col + 1, &sensors.right),
            (pos.row - 1, pos.col - 1, &sensors.up_left),
            (pos.row - 1, pos.col + 1, &sensors.up_right),
            (pos.row + 1, pos.col - 1, &sensors.down_left),
            (pos.row + 1, pos.col + 1, &sensors.down_right),
        ];

        for (row, col, sensor_state) in sensor_data {
            let cell_pos = UnboundedPosition::new(row, col);
            let cell = match sensor_state {
                SensorState::Free => Cell::Free,
                SensorState::Blocked => Cell::Blocked,
                SensorState::Target => Cell::Target,
            };
            self.set(cell_pos, cell);
        }
    }
}

impl Default for UnboundedMaze {
    fn default() -> Self {
        Self::new()
    }
}
