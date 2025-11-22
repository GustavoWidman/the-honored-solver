mod algorithms;
mod cli;
mod logging;
mod maze;
mod ros;
mod solvers;

use std::process::{Command, Stdio};

use clap::Parser;
use eyre::Result;
use log::{debug, info};

use algorithms::{exploration, pathfinding};
use cli::{Args, BenchmarkMode, Command as CliCommand, ExplorationAlgorithm, PathfindingAlgorithm};
use logging::Logger;
use ros::ROSInterface;
use solvers::{BlindSolver, OmniscientSolver};

#[tokio::main]
#[macros::with_node]
async fn main() -> Result<()> {
    let args = Args::parse();
    Logger::init(args.verbosity);

    let mut cg_command = Command::new("ros2")
        .arg("run")
        .arg("cg")
        .arg("maze")
        .args(build_cg_args(&args))
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()?;

    info!("throughout heaven and earth, i alone am the honored solver.");

    let ros = ROSInterface::new(&mut node)?;
    start_node(node);
    ros.init().await?;

    match args.command {
        CliCommand::Omniscient { algorithm } => {
            run_omniscient_solver(ros, algorithm, args.delay).await?;
        }
        CliCommand::Blind { algorithm } => {
            run_blind_solver(ros, algorithm, args.delay).await?;
        }
        CliCommand::Benchmark { mode } => match mode {
            BenchmarkMode::Omniscient => {
                run_omniscient_benchmark(ros, args.delay).await?;
            }
            BenchmarkMode::Blind => {
                run_blind_benchmark(ros, args.delay).await?;
            }
        },
    }

    cg_command.wait()?;
    Ok(())
}

fn build_cg_args(args: &Args) -> Vec<String> {
    let mut cg_args = vec!["--".to_string()];

    if args.generate {
        info!("generating new maze");
        cg_args.push("--generate".to_string());
    } else if let Some(ref map_name) = args.map_name {
        info!("loading map: {}", map_name);
        cg_args.extend(["--map".to_string(), map_name.clone()]);
    } else {
        debug!("loading random maze");
    }

    cg_args
}

// ========== Omniscient Solvers ==========

async fn solve_omniscient(
    ros: std::sync::Arc<ROSInterface>,
    algorithm: PathfindingAlgorithm,
    delay: u64,
) -> Result<pathfinding::PathResult> {
    match algorithm {
        PathfindingAlgorithm::AStar => {
            OmniscientSolver::new(pathfinding::AStar, delay)
                .solve(ros)
                .await
        }
        PathfindingAlgorithm::Dijkstra => {
            OmniscientSolver::new(pathfinding::Dijkstra, delay)
                .solve(ros)
                .await
        }
        PathfindingAlgorithm::DFS => {
            OmniscientSolver::new(pathfinding::DFS, delay)
                .solve(ros)
                .await
        }
    }
}

async fn run_omniscient_solver(
    ros: std::sync::Arc<ROSInterface>,
    algorithm: PathfindingAlgorithm,
    delay: u64,
) -> Result<()> {
    info!("solving with {}", algorithm.name());
    if delay > 0 {
        debug!("delay: {}ms", delay);
    }

    let result = solve_omniscient(ros, algorithm, delay).await?;
    print_result(&result, algorithm.name());
    Ok(())
}

async fn run_omniscient_benchmark(ros: std::sync::Arc<ROSInterface>, delay: u64) -> Result<()> {
    info!("benchmarking omniscient algorithms");

    let mut completed_results = Vec::new();

    for (i, algorithm) in PathfindingAlgorithm::all().enumerate() {
        info!("testing {}", algorithm.name());

        if i > 0 {
            debug!("resetting maze");
            ros.reset(false, String::new()).await?;
            tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
        }

        match solve_omniscient(ros.clone(), algorithm, delay).await {
            Ok(result) => {
                print_result(&result, algorithm.name());
                completed_results.push((algorithm.name(), result));
            }
            Err(e) => {
                log::error!("{} failed: {}", algorithm.name(), e);
            }
        }
    }

    print_benchmark_summary(&completed_results);
    Ok(())
}

// ========== Blind Solvers ==========

async fn solve_blind(
    ros: std::sync::Arc<ROSInterface>,
    algorithm: ExplorationAlgorithm,
    delay: u64,
) -> Result<pathfinding::PathResult> {
    match algorithm {
        ExplorationAlgorithm::WallFollower => {
            let mut solver = BlindSolver::new(exploration::WallFollower::new(), delay);
            solver.solve(ros).await
        }
        ExplorationAlgorithm::RecursiveBacktracker => {
            let mut solver = BlindSolver::new(exploration::RecursiveBacktracker::new(), delay);
            solver.solve(ros).await
        }
    }
}

async fn run_blind_solver(
    ros: std::sync::Arc<ROSInterface>,
    algorithm: ExplorationAlgorithm,
    delay: u64,
) -> Result<()> {
    info!("exploring with {}", algorithm.name());
    if delay > 0 {
        debug!("delay: {}ms", delay);
    }

    let result = solve_blind(ros, algorithm, delay).await?;
    print_result(&result, algorithm.name());
    Ok(())
}

async fn run_blind_benchmark(ros: std::sync::Arc<ROSInterface>, delay: u64) -> Result<()> {
    info!("benchmarking blind algorithms");

    let mut completed_results = Vec::new();

    for (i, algorithm) in ExplorationAlgorithm::all().enumerate() {
        info!("testing {}", algorithm.name());

        if i > 0 {
            debug!("resetting maze");
            ros.reset(false, String::new()).await?;
            tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
        }

        match solve_blind(ros.clone(), algorithm, delay).await {
            Ok(result) => {
                print_result(&result, algorithm.name());
                completed_results.push((algorithm.name(), result));
            }
            Err(e) => {
                log::error!("{} failed: {}", algorithm.name(), e);
            }
        }
    }

    print_benchmark_summary(&completed_results);
    Ok(())
}

// ========== Utilities ==========

fn print_result(result: &pathfinding::PathResult, _algorithm_name: &str) {
    info!(
        "finished in {} steps ({:?})",
        result.steps, result.total_time
    );
    debug!("planning: {:?}", result.planning_time);
    debug!("execution: {:?}", result.execution_time);
}

fn print_benchmark_summary(results: &[(&str, pathfinding::PathResult)]) {
    info!("\nbenchmark results:");
    info!(
        "{:<20} {:>8}  {:>12}  {:>12}",
        "algorithm", "steps", "plan", "total"
    );
    info!("{:-<60}", "");

    for (name, result) in results {
        info!(
            "{:<20} {:>8}  {:>12?}  {:>12?}",
            name, result.steps, result.planning_time, result.total_time,
        );
    }

    if let Some((name, result)) = results.iter().min_by_key(|(_, r)| r.steps) {
        info!("\nbest: {} ({} steps)", name, result.steps);
    }

    if let Some((name, result)) = results.iter().min_by_key(|(_, r)| r.total_time) {
        info!("fastest: {} ({:?})", name, result.total_time);
    }
}
