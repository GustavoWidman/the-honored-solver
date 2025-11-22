use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::{
    maze::{BoundedMaze, Position},
    ros::types::MoveDirection,
};

use super::traits::PathfindingAlgorithm;

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: Position,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .cmp(&self.cost)
            .then_with(|| self.position.row.cmp(&other.position.row))
            .then_with(|| self.position.col.cmp(&other.position.col))
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct Dijkstra;

impl PathfindingAlgorithm for Dijkstra {
    fn find_path(
        &self,
        maze: &BoundedMaze,
        start: Position,
        target: Position,
    ) -> Option<Vec<MoveDirection>> {
        let mut heap = BinaryHeap::new();
        let mut distances: HashMap<Position, usize> = HashMap::new();
        let mut came_from: HashMap<Position, (Position, MoveDirection)> = HashMap::new();

        heap.push(State {
            cost: 0,
            position: start,
        });
        distances.insert(start, 0);

        while let Some(State { cost, position }) = heap.pop() {
            if position == target {
                return Some(reconstruct_path(&came_from, start, target));
            }

            if cost > *distances.get(&position).unwrap_or(&usize::MAX) {
                continue;
            }

            for (neighbor, direction) in maze.neighbors(position) {
                let new_cost = cost + 1;
                let current_dist = distances.get(&neighbor).copied().unwrap_or(usize::MAX);

                if new_cost < current_dist {
                    distances.insert(neighbor, new_cost);
                    came_from.insert(neighbor, (position, direction));
                    heap.push(State {
                        cost: new_cost,
                        position: neighbor,
                    });
                }
            }
        }

        None
    }

    fn name(&self) -> &'static str {
        "Dijkstra"
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
