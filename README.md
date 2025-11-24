# the honored solver

a rust-based maze solver that interfaces with ros2 to solve mazes using either omniscient pathfinding or blind exploration algorithms.

## overview

this project implements two distinct approaches to maze solving:

- **omniscient mode**: retrieves the complete maze map upfront, then uses pathfinding algorithms to compute the optimal path before execution
- **blind mode**: explores the maze using only sensor data (8-directional sensors), building a map dynamically as it moves through an unbounded coordinate system

the solver communicates with a ros2 maze environment (`culling_games`) to control a robot and receive sensor feedback.

## how it works

### omniscient solvers

omniscient solvers call the `/get_map` ros2 service to obtain the full maze layout at the start. they then compute a complete path from start to target using classical pathfinding algorithms before sending any movement commands.

the maze is represented as a bounded grid where:
- each cell can be: free, blocked, target, or robot
- the robot moves in 4 cardinal directions (up, down, left, right)
- pathfinding algorithms find the shortest or valid path to the target

### blind solvers

blind solvers never call `/get_map`. instead, they:
1. subscribe to the `/culling_games/robot_sensors` topic to receive 8-directional sensor data
2. use an unbounded coordinate system (signed integers) that grows infinitely as new areas are explored
3. build a sparse hashmap-based representation of discovered cells
4. apply exploration strategies to navigate toward the target

the sensors detect:
- `blocked`: wall or obstacle
- `free`: empty walkable space
- `target`: goal position

## algorithms

### pathfinding (omniscient)

these algorithms work on a complete known map:

| algorithm | description | characteristics |
|-----------|-------------|-----------------|
| **a*** | uses manhattan distance heuristic | optimal path, efficient search |
| **dijkstra** | uniform cost search | optimal path, explores more nodes |
| **dfs** | depth-first search | finds a path, not necessarily optimal |

### exploration (blind)

these algorithms discover the maze through sensor-based navigation:

| algorithm | description | characteristics |
|-----------|-------------|-----------------|
| **wall follower** | left-hand rule maze traversal | keeps left hand on wall, guaranteed to find exit in simply-connected mazes |
| **recursive backtracker** | dfs-based exploration with explicit backtracking | explores all unvisited neighbors, backtracks using bfs when stuck |

both exploration algorithms:
- detect when the target appears in sensor range
- attempt to move directly toward the target once spotted (if in cardinal direction)
- continue exploring if the target is not yet visible

## project structure

```
.
├── solver/                    # main solver binary
│   └── src/
│       ├── algorithms/        # algorithm implementations
│       │   ├── exploration/   # blind exploration algorithms
│       │   └── pathfinding/   # omniscient pathfinding algorithms
│       ├── maze/              # maze representations (bounded and unbounded)
│       ├── ros/               # ros2 interface and types
│       ├── solvers/           # solver wrappers (omniscient and blind)
│       ├── cli.rs             # command-line interface
│       ├── logging.rs         # custom logger with colored output
│       └── main.rs            # entry point
├── macros/                    # procedural macros for ros2 node setup and general ros2 QoL
└── flake.nix                  # nix development environment
```

## setup

### using nix shell (recommended)

this project uses [nix flakes](https://nixos.org/manual/nix/stable/command-ref/new-cli/nix3-flake.html) with [micromamba](https://mamba.readthedocs.io/en/latest/user_guide/micromamba.html) to manage ros2 dependencies via [robostack](https://robostack.github.io/).

**requirements:**
- nix with flakes enabled
- sudo access (for creating `/opt/micromamba`)

**steps:**

1. clone the repository:
```bash
git clone https://github.com/GustavoWidman/the-honored-solver.git
cd the-honored-solver
```

2. enter the nix development shell:
```bash
nix develop
```

the shell hook will automatically:
- create `/opt/micromamba` directory (requires sudo, prompts for password)
- create a micromamba environment named `ros_env`
- install ros2 humble and all dependencies via robostack
- clone the `culling_games` repository if it does not exist
- build the `culling_games` package with colcon
- source the ros2 setup scripts

3. build the solver:
```bash
cargo build --release
```

the nix shell provides these custom commands:
- `ros-install [packages...]`: install additional ros2 packages
- `ros-update`: update all packages in the ros environment

### without nix shell

if you prefer not to use nix, you need to manually set up the environment:

**requirements:**
- rust toolchain (1.75+ should work, 2024 edition is needed)
- ros2 humble
- the `culling_games` package

**steps:**

1. install ros2 humble following the [official instructions](https://docs.ros.org/en/humble/Installation.html)

2. clone and build the culling games package:
```bash
git clone https://github.com/rmnicola/culling_games.git
cd culling_games
colcon build
source install/setup.bash
cd ..
```

3. clone the repo:
```bash
git clone https://github.com/GustavoWidman/the-honored-solver.git
cd the-honored-solver
```

4. ensure ros2 is sourced before running or building:
```bash
source /opt/ros/humble/setup.bash

cd culling_games
colcon build
source install/setup.bash
cd ..
```

5. build or run
```bash
cargo build --release
./target/release/solver --help

# or

cargo run --release -- --help
```

## usage

first, build the release binary:

```bash
cargo build --release
```

the solver uses a subcommand-based cli:

### omniscient mode

solve a maze with complete map knowledge:

```bash
# using a-star (or 'a-star')
./target/release/solver omniscient astar

# using dijkstra
./target/release/solver omniscient dijkstra

# using dfs
./target/release/solver omniscient dfs
```

### blind mode

explore a maze using only sensors:

```bash
# using wall follower
./target/release/solver blind wall-follower

# using recursive backtracker
./target/release/solver blind recursive-backtracker
```

### benchmark mode

run all algorithms in a category and compare results:

```bash
# benchmark all omniscient algorithms
./target/release/solver benchmark omniscient

# benchmark all blind algorithms
./target/release/solver benchmark blind
```

benchmark output includes:
- number of steps taken
- planning time (computation)
- total execution time
- comparison showing best and fastest algorithms

### options

```
options:
  -h, --help                    print help information
  -v, --verbosity <LEVEL>       log level: off, error, warn, info, debug, trace [default: info]
  -d, --delay <MS>              delay between moves in milliseconds [default: 0]
  --map-name <NAME>             load a specific map from culling_games/src/cg/maps
  -g, --generate                generate a new random maze
```

**examples:**

```bash
# solve with specific map
./target/release/solver --map-name "test" omniscient astar # uses culling_games/src/cg/maps/test.csv, specifying "test.csv" seems to work for that same file as well...

# generate random maze and explore blindly
./target/release/solver -g blind wall-follower

# benchmark with 100ms delay between moves (for visualization)
./target/release/solver -d 100 benchmark blind

# enable debug logging
./target/release/solver -v debug blind recursive-backtracker
```

## logging

the solver uses a custom colored logger with two main levels:

- **info**: minimal output showing key events (target found, completion, results)
- **debug**: verbose output including every move, backtracking, sensor data

example info output:
```
2024-11-24 15:32:10.123456 INFO  loading map: maze01
2025-11-24 04:17:03.381527 INFO  throughout heaven and earth, i alone am the honored solver.
2024-11-24 15:32:10.234567 INFO  solving with A*
2024-11-24 15:32:10.345678 INFO  planned 42 steps in 2ms
2024-11-24 15:32:11.456789 INFO  reached target
2024-11-24 15:32:11.567890 INFO  finished in 42 steps (1.2s)
```

example debug output adds:
```
2024-11-24 15:32:10.123456 DEBUG fetching maze map
2024-11-24 15:32:10.234567 DEBUG 29x29 maze: (0, 0) → (28, 28)
2024-11-24 15:32:10.345678 DEBUG step 1/42: Up
2024-11-24 15:32:10.456789 DEBUG step 2/42: Right
...
```

## demonstration

<!-- video/explanation section - to be filled in later -->

## license

this project is licensed under the MIT license - see the [LICENSE](LICENSE) file for details.

## acknowledgments

- [robostack](https://robostack.github.io/) for providing ros2 packages via conda-forge
- [culling games](https://github.com/rmnicola/culling_games) for the maze environment
- [r2r](https://github.com/sequenceplanner/r2r) for rust ros2 bindings
