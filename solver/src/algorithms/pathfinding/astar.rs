use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::{
    maze::{BoundedMaze, Position},
    ros::types::MoveDirection,
};

use super::traits::PathfindingAlgorithm;

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    f_score: usize,
    g_score: usize,
    position: Position,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .cmp(&self.f_score)
            .then_with(|| self.position.row.cmp(&other.position.row))
            .then_with(|| self.position.col.cmp(&other.position.col))
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct AStar;

impl PathfindingAlgorithm for AStar {
    fn find_path(
        &self,
        maze: &BoundedMaze,
        start: Position,
        target: Position,
    ) -> Option<Vec<MoveDirection>> {
        let mut open_set = BinaryHeap::new();
        let mut came_from: HashMap<Position, (Position, MoveDirection)> = HashMap::new();
        let mut g_scores: HashMap<Position, usize> = HashMap::new();
        let mut closed_set: HashSet<Position> = HashSet::new();

        g_scores.insert(start, 0);
        open_set.push(State {
            f_score: start.manhattan_distance(target),
            g_score: 0,
            position: start,
        });

        while let Some(State {
            position, g_score, ..
        }) = open_set.pop()
        {
            if position == target {
                return Some(reconstruct_path(&came_from, start, target));
            }

            if closed_set.contains(&position) {
                continue;
            }

            closed_set.insert(position);

            if g_score > *g_scores.get(&position).unwrap_or(&usize::MAX) {
                continue;
            }

            for (neighbor, direction) in maze.neighbors(position) {
                if closed_set.contains(&neighbor) {
                    continue;
                }

                let tentative_g = g_score + 1;
                let current_g = g_scores.get(&neighbor).copied().unwrap_or(usize::MAX);

                if tentative_g < current_g {
                    g_scores.insert(neighbor, tentative_g);
                    came_from.insert(neighbor, (position, direction));

                    let f_score = tentative_g + neighbor.manhattan_distance(target);
                    open_set.push(State {
                        f_score,
                        g_score: tentative_g,
                        position: neighbor,
                    });
                }
            }
        }

        None
    }

    fn name(&self) -> &'static str {
        "A*"
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
