use clap::{Parser, Subcommand, ValueEnum};
use log::LevelFilter;

#[derive(Parser, Debug)]
#[command(name = "the-honored-solver")]
#[command(about = "Maze solver with multiple strategies")]
pub struct Args {
    /// Sets the logger's verbosity level
    #[arg(short, long, value_name = "VERBOSITY", default_value_t = LevelFilter::Info)]
    pub verbosity: LevelFilter,

    /// Specific map file to load (from culling_games/src/cg/maps)
    #[arg(long)]
    pub map_name: Option<String>,

    /// Generate a completely new random maze
    #[arg(short, long)]
    pub generate: bool,

    /// Delay between moves in milliseconds (0 = no delay)
    #[arg(short, long, default_value_t = 0)]
    pub delay: u64,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand, Debug)]
pub enum Command {
    /// Omniscient mode: Get full map, then plan optimal path
    Omniscient {
        /// Pathfinding algorithm to use
        #[arg(value_enum)]
        algorithm: PathfindingAlgorithm,
    },

    /// Blind mode: Explore using only sensors (no map knowledge)
    Blind {
        /// Exploration algorithm to use for discovery
        #[arg(value_enum)]
        exploration: ExplorationAlgorithm,

        /// Pathfinding algorithm to use for optimal route
        #[arg(value_enum)]
        pathfinding: PathfindingAlgorithm,
    },

    /// Benchmark mode: Run all algorithms and compare performance
    Benchmark {
        /// Mode to benchmark
        #[command(subcommand)]
        mode: BenchmarkMode,
    },
}

#[derive(Subcommand, Debug)]
pub enum BenchmarkMode {
    /// Benchmark all omniscient pathfinding algorithms
    Omniscient,

    /// Benchmark all blind exploration algorithms
    Blind,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum PathfindingAlgorithm {
    /// A* algorithm with Manhattan distance heuristic
    #[value(name = "astar", alias = "a-star")]
    AStar,

    /// Dijkstra's shortest path algorithm
    Dijkstra,

    /// Depth-First Search
    #[value(name = "dfs")]
    #[allow(clippy::upper_case_acronyms)]
    DFS,
}

impl PathfindingAlgorithm {
    pub fn all() -> impl Iterator<Item = Self> {
        [Self::AStar, Self::Dijkstra, Self::DFS].into_iter()
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::AStar => "A*",
            Self::Dijkstra => "Dijkstra",
            Self::DFS => "DFS",
        }
    }
}

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum ExplorationAlgorithm {
    /// Wall follower using left-hand rule
    #[value(name = "wall-follower")]
    WallFollower,

    /// Recursive backtracker (DFS-based exploration)
    #[value(name = "recursive-backtracker")]
    RecursiveBacktracker,
}

impl ExplorationAlgorithm {
    pub fn all() -> impl Iterator<Item = Self> {
        [Self::WallFollower, Self::RecursiveBacktracker].into_iter()
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::WallFollower => "Wall Follower",
            Self::RecursiveBacktracker => "Recursive Backtracker",
        }
    }
}
