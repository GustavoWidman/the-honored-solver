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

blind solvers never call `/get_map`. instead, they operate in four distinct steps:

**known constraints:**
- map is always 29×29 with a 1-tile thick wall around the perimeter
- robot always spawns at position (1, 1) in maze coordinates, mapped to (0, 0) in our unbounded coordinate system
- target is always at the center of the maze

**step 1: full maze exploration**
1. subscribe to the `/culling_games/robot_sensors` topic to receive 8-directional sensor data
2. use an unbounded coordinate system (signed integers) that grows infinitely as new areas are explored
3. build a sparse hashmap-based representation of discovered cells
4. apply exploration strategies to **fully map the entire maze**
5. exploration continues even after spotting the target, until the exploration algorithm completes
6. robot **never reaches the target** during this phase

**step 2: reset robot state**
1. reset the robot position to origin without changing the discovered map
2. the fully explored maze map is retained

**step 3: optimal path planning**
1. convert the discovered unbounded maze to a bounded representation
2. use a pathfinding algorithm to compute the optimal path to the target

**step 4: execute optimal path**
1. execute the computed optimal path
2. robot reaches the target

the sensors detect:
- `blocked`: wall or obstacle
- `free`: empty walkable space
- `target`: goal position

this approach ensures complete maze knowledge before pathfinding, allowing for globally optimal solutions while still maintaining the "blind" constraint of not calling `/get_map`.

## algorithms

### pathfinding (omniscient)

these algorithms work on a complete known map:

| algorithm | description | characteristics |
|-----------|-------------|-----------------|
| **a*** | uses manhattan distance heuristic | optimal path, efficient search |
| **dijkstra** | uniform cost search | optimal path, explores more nodes |
| **dfs** | depth-first search | finds a path, not necessarily optimal |

### blind mode (exploration + pathfinding)

blind solvers combine an **exploration algorithm** (step 1) with a **pathfinding algorithm** (step 3):

**exploration algorithms** fully map the maze through sensor-based navigation:

| algorithm | description | characteristics |
|-----------|-------------|-----------------|
| **wall follower** | left-hand rule maze traversal | follows walls until returning to start, explores perimeter and accessible loops |
| **recursive backtracker** | dfs-based exploration with explicit backtracking | systematically explores all reachable cells, backtracks using bfs when stuck |

all exploration algorithms:
- detect when the target appears in sensor range (for later pathfinding)
- continue exploring after spotting the target
- complete full maze exploration before stopping
- never reach the target during exploration phase

**pathfinding algorithms** compute the optimal path on the fully discovered map:

after full exploration completes, the same pathfinding algorithms from omniscient mode (a*, dijkstra, dfs) are used to find the optimal route through the discovered maze. the solver then resets and executes this optimal path.

this combination means blind solvers test all permutations: 2 exploration algorithms × 3 pathfinding algorithms = 6 total combinations.

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

explore a maze using only sensors, combining an exploration algorithm with a pathfinding algorithm:

```bash
# syntax: blind <exploration> <pathfinding>

# wall follower exploration + a* pathfinding
./target/release/solver blind wall-follower astar

# recursive backtracker exploration + dijkstra pathfinding
./target/release/solver blind recursive-backtracker dijkstra

# wall follower exploration + dfs pathfinding
./target/release/solver blind wall-follower dfs
```

available exploration algorithms:
- `wall-follower` - left-hand rule maze traversal
- `recursive-backtracker` - dfs-based exploration with backtracking

available pathfinding algorithms:
- `astar` (or `a-star`) - manhattan distance heuristic
- `dijkstra` - uniform cost search
- `dfs` - depth-first search

### benchmark mode

run all algorithms in a category and compare results:

```bash
# benchmark all omniscient algorithms (3 algorithms)
./target/release/solver benchmark omniscient

# benchmark all blind algorithms (6 combinations)
./target/release/solver benchmark blind
```

benchmark output includes:
- number of steps taken (exploration + execution for blind mode)
- planning time (computation only)
- total execution time (including robot movements)
- comparison showing best (fewest steps) and fastest (shortest time) algorithms

**note:** blind mode benchmarks test all 6 combinations of exploration + pathfinding algorithms (wall follower + a*, wall follower + dijkstra, etc.)

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
# uses culling_games/src/cg/maps/test.csv, although
# specifying "test.csv" seems to work for that same file as well...
./target/release/solver --map-name "test" omniscient astar

# generate random maze and explore blindly
./target/release/solver -g blind wall-follower astar

# benchmark with 100ms delay between moves (for visualization)
./target/release/solver -d 100 benchmark blind

# enable debug logging
./target/release/solver -v debug blind recursive-backtracker dijkstra
```

## logging

the solver uses a custom colored logger with two main levels:

- **info**: minimal output showing key events (target found, completion, results)
- **debug**: verbose output including every move, backtracking, sensor data

example omniscient info output:
```
2024-11-24 15:32:10.123456 INFO  loading map: maze01
2025-11-24 04:17:03.381527 INFO  throughout heaven and earth, i alone am the honored solver.
2024-11-24 15:32:10.234567 INFO  solving with A*
2024-11-24 15:32:10.345678 INFO  planned 42 steps in 2ms
2024-11-24 15:32:11.456789 INFO  reached target
2024-11-24 15:32:11.567890 INFO  finished in 42 steps (1.2s)
```

example blind info output:
```
2024-11-24 15:32:10.123456 INFO  loading map: maze01
2025-11-24 04:17:03.381527 INFO  throughout heaven and earth, i alone am the honored solver.
2024-11-24 15:32:10.234567 INFO  exploring with Wall Follower + A*
2024-11-24 15:32:10.345678 INFO  starting at origin
2024-11-24 15:32:10.456789 INFO  phase 1: exploring maze with Wall Follower (Left-Hand Rule)
2024-11-24 15:32:15.123456 INFO  target spotted at (13, 13)
2024-11-24 15:32:35.234567 INFO  exploration complete after 450 steps
2024-11-24 15:32:35.345678 INFO  exploration complete: found target at (13, 13) in 450 steps
2024-11-24 15:32:35.456789 INFO  phase 2: planning optimal path with A*
2024-11-24 15:32:35.567890 INFO  planned optimal path: 26 steps
2024-11-24 15:32:35.678901 INFO  resetting maze and executing optimal path
2024-11-24 15:32:36.789012 INFO  reached target
2024-11-24 15:32:36.890123 INFO  total: 450 exploration + 26 execution = 476 steps
2024-11-24 15:32:36.901234 INFO  finished in 476 steps (26.5s)
```

example debug output adds move-by-move details:
```
2024-11-24 15:32:10.123456 DEBUG fetching maze map
2024-11-24 15:32:10.234567 DEBUG 29x29 maze: (0, 0) → (28, 28)
2024-11-24 15:32:10.345678 DEBUG step 1/42: Up
2024-11-24 15:32:10.456789 DEBUG exploration step 1: Down from (0, 0)
2024-11-24 15:32:10.567890 DEBUG cache hit! using cached sensors for (5, 3)
2024-11-24 15:32:10.678901 DEBUG backtracking to (4, 3)
...
```

## demonstration

https://github.com/user-attachments/assets/7b057301-a6c0-48e8-931a-c20a72b59605

## license

this project is licensed under the MIT license - see the [LICENSE](LICENSE) file for details.

## acknowledgments

- [robostack](https://robostack.github.io/) for providing ros2 packages via conda-forge
- [culling games](https://github.com/rmnicola/culling_games) for the maze environment
- [r2r](https://github.com/sequenceplanner/r2r) for rust ros2 bindings
