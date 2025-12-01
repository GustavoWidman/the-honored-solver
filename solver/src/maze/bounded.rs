use super::{cell::Cell, position::Position};
use crate::ros::types::MoveDirection;

#[derive(Debug, Clone)]
pub struct Maze {
    grid: Vec<Cell>,
    width: usize,
    height: usize,
}

impl Maze {
    pub fn from_flattened(flattened: Vec<String>, shape: Vec<u8>) -> eyre::Result<Self> {
        if shape.len() != 2 {
            eyre::bail!("invalid shape: expected [height, width], got {:?}", shape);
        }

        let height = shape[0] as usize;
        let width = shape[1] as usize;

        if flattened.len() != width * height {
            eyre::bail!(
                "grid size mismatch: expected {}, got {}",
                width * height,
                flattened.len()
            );
        }

        let grid = flattened.iter().map(|s| Cell::from_str(s)).collect();

        Ok(Self {
            grid,
            width,
            height,
        })
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn bounds(&self) -> (usize, usize) {
        (self.height, self.width)
    }

    pub fn get(&self, pos: Position) -> Option<Cell> {
        if pos.row < self.height && pos.col < self.width {
            Some(self.grid[pos.to_index(self.width)])
        } else {
            None
        }
    }

    pub fn is_walkable(&self, pos: Position) -> bool {
        self.get(pos).is_some_and(|cell| cell.is_walkable())
    }

    pub fn find_robot(&self) -> Option<Position> {
        self.grid
            .iter()
            .position(|&cell| cell == Cell::Robot)
            .map(|idx| Position::from_index(idx, self.width))
    }

    pub fn find_target(&self) -> Option<Position> {
        self.grid
            .iter()
            .position(|&cell| cell == Cell::Target)
            .map(|idx| Position::from_index(idx, self.width))
    }

    pub fn neighbors(&self, pos: Position) -> Vec<(Position, MoveDirection)> {
        pos.neighbors(self.bounds())
            .into_iter()
            .filter(|(p, _)| self.is_walkable(*p))
            .collect()
    }
}
