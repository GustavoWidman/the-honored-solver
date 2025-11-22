use std::ops::Add;

use crate::ros::types::MoveDirection;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Position {
    pub row: usize,
    pub col: usize,
}

impl Position {
    pub const fn new(row: usize, col: usize) -> Self {
        Self { row, col }
    }

    pub fn to_index(self, width: usize) -> usize {
        self.row * width + self.col
    }

    pub fn from_index(index: usize, width: usize) -> Self {
        Self::new(index / width, index % width)
    }

    pub fn manhattan_distance(self, other: Self) -> usize {
        self.row.abs_diff(other.row) + self.col.abs_diff(other.col)
    }

    pub fn move_in_direction(
        self,
        direction: MoveDirection,
        bounds: (usize, usize),
    ) -> Option<Self> {
        let (height, width) = bounds;
        match direction {
            MoveDirection::Up if self.row > 0 => Some(Self::new(self.row - 1, self.col)),
            MoveDirection::Down if self.row < height - 1 => Some(Self::new(self.row + 1, self.col)),
            MoveDirection::Left if self.col > 0 => Some(Self::new(self.row, self.col - 1)),
            MoveDirection::Right if self.col < width - 1 => Some(Self::new(self.row, self.col + 1)),
            _ => None,
        }
    }

    pub fn neighbors(self, bounds: (usize, usize)) -> Vec<(Self, MoveDirection)> {
        [
            MoveDirection::Up,
            MoveDirection::Down,
            MoveDirection::Left,
            MoveDirection::Right,
        ]
        .into_iter()
        .filter_map(|dir| self.move_in_direction(dir, bounds).map(|pos| (pos, dir)))
        .collect()
    }
}

impl Add<(isize, isize)> for Position {
    type Output = Option<Self>;

    fn add(self, (dr, dc): (isize, isize)) -> Self::Output {
        let new_row = self.row as isize + dr;
        let new_col = self.col as isize + dc;

        if new_row >= 0 && new_col >= 0 {
            Some(Self::new(new_row as usize, new_col as usize))
        } else {
            None
        }
    }
}
