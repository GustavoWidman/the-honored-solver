use std::collections::{HashMap, HashSet};

use crate::{
    maze::{BoundedMaze, Position},
    ros::types::MoveDirection,
};

use super::traits::PathfindingAlgorithm;

#[allow(clippy::upper_case_acronyms)]
pub struct DFS;

impl PathfindingAlgorithm for DFS {
    fn find_path(
        &self,
        maze: &BoundedMaze,
        start: Position,
        target: Position,
    ) -> Option<Vec<MoveDirection>> {
        let mut visited = HashSet::new();
        let mut came_from = HashMap::new();
        let mut stack = vec![start];
        visited.insert(start);

        while let Some(current) = stack.pop() {
            if current == target {
                return Some(reconstruct_path(&came_from, start, target));
            }

            for (neighbor, direction) in maze.neighbors(current) {
                if !visited.contains(&neighbor) {
                    visited.insert(neighbor);
                    came_from.insert(neighbor, (current, direction));
                    stack.push(neighbor);
                }
            }
        }

        None
    }

    fn name(&self) -> &'static str {
        "DFS"
    }
}

fn reconstruct_path(
    came_from: &HashMap<Position, (Position, MoveDirection)>,
    start: Position,
    target: Position,
) -> Vec<MoveDirection> {
    let mut path = Vec::new();
    let mut current = target;

    while current != start {
        if let Some(&(prev, direction)) = came_from.get(&current) {
            path.push(direction);
            current = prev;
        } else {
            break;
        }
    }

    path.reverse();
    path
}
