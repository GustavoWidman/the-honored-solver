use std::collections::{HashSet, VecDeque};

use crate::{
    maze::{UnboundedMaze, UnboundedPosition},
    ros::types::{MoveDirection, SensorsStates},
};

use super::traits::ExplorationAlgorithm;

/// dfs-based exploration with backtracking
pub struct RecursiveBacktracker {
    visited: HashSet<UnboundedPosition>,
    path_stack: VecDeque<UnboundedPosition>,
}

impl RecursiveBacktracker {
    pub fn new() -> Self {
        Self {
            visited: HashSet::new(),
            path_stack: VecDeque::new(),
        }
    }

    fn get_unvisited_neighbors(
        &self,
        current: UnboundedPosition,
        sensors: &SensorsStates,
    ) -> Vec<(UnboundedPosition, MoveDirection)> {
        use crate::ros::types::SensorState;

        let mut unvisited = Vec::new();

        let directions = [
            (sensors.up, MoveDirection::Up, current.row - 1, current.col),
            (
                sensors.down,
                MoveDirection::Down,
                current.row + 1,
                current.col,
            ),
            (
                sensors.left,
                MoveDirection::Left,
                current.row,
                current.col - 1,
            ),
            (
                sensors.right,
                MoveDirection::Right,
                current.row,
                current.col + 1,
            ),
        ];

        for (state, direction, row, col) in directions {
            // treat target as blocked during exploration - we don't want to reach it yet
            if matches!(state, SensorState::Free) {
                let pos = UnboundedPosition::new(row, col);
                if !self.visited.contains(&pos) {
                    unvisited.push((pos, direction));
                }
            }
        }

        unvisited
    }

    /// bfs for navigating in already-discovered maze
    fn find_path_bfs(
        &self,
        maze: &UnboundedMaze,
        start: UnboundedPosition,
        goal: UnboundedPosition,
    ) -> Option<MoveDirection> {
        use std::collections::{HashMap, VecDeque};

        let mut queue = VecDeque::new();
        let mut visited = HashSet::new();
        let mut came_from: HashMap<UnboundedPosition, (UnboundedPosition, MoveDirection)> =
            HashMap::new();

        queue.push_back(start);
        visited.insert(start);

        while let Some(current) = queue.pop_front() {
            if current == goal {
                let mut pos = goal;
                while let Some((prev, dir)) = came_from.get(&pos) {
                    if *prev == start {
                        return Some(*dir);
                    }
                    pos = *prev;
                }
                return None;
            }

            for (neighbor, direction) in maze.neighbors(current) {
                if visited.insert(neighbor) {
                    came_from.insert(neighbor, (current, direction));
                    queue.push_back(neighbor);
                }
            }
        }

        None
    }
}

impl ExplorationAlgorithm for RecursiveBacktracker {
    fn next_move(
        &mut self,
        current_pos: UnboundedPosition,
        sensors: &SensorsStates,
        maze: &UnboundedMaze,
    ) -> eyre::Result<Option<MoveDirection>> {
        self.visited.insert(current_pos);

        let unvisited_neighbors = self.get_unvisited_neighbors(current_pos, sensors);

        if !unvisited_neighbors.is_empty() {
            let (_next_pos, direction) = unvisited_neighbors[0];
            self.path_stack.push_back(current_pos);
            return Ok(Some(direction));
        }

        if let Some(backtrack_target) = self.path_stack.pop_back() {
            log::debug!(
                "backtracking to ({}, {})",
                backtrack_target.row,
                backtrack_target.col
            );

            if let Some(first_move) = self.find_path_bfs(maze, current_pos, backtrack_target) {
                return Ok(Some(first_move));
            }
        }

        Ok(None)
    }

    fn name(&self) -> &'static str {
        "Recursive Backtracker"
    }

    fn reset(&mut self) {
        self.visited.clear();
        self.path_stack.clear();
    }
}
