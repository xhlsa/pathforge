# pathforge
[![Crates.io](https://img.shields.io/crates/v/pathforge.svg)](https://crates.io/crates/pathforge)
[![Docs](https://docs.rs/pathforge/badge.svg)](https://docs.rs/pathforge)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](#license)

High-performance, game-ready pathfinding for Rust with zero-allocation hot paths and game-loop friendly APIs.

## Features
- A* with configurable heuristics (Manhattan, Euclidean, Diagonal, Zero for Dijkstra behavior)
- Jump Point Search (uniform-cost grids) for 10x+ speedups on empty/low-obstacle maps
- Flow fields for RTS-style crowd steering (one compute, O(1) queries)
- Frame budgeting (`BudgetedPathfinder`) with partial results between frames
- Path caching with TTL and LRU-like eviction (`astar_with_cache`)
- Path smoothing (string-pulling) to remove stair-step artifacts
- Grid2D graph with diagonal modes and per-cell costs; trait-based `Graph` for custom graphs
- Dynamic obstacles (block/unblock at runtime) and weighted terrain

## Quick start: frame-budgeted search
Copy-paste friendly game-loop example (see `examples/frame_budget.rs`):

```rust
use std::time::Duration;
use pathforge::algorithms::astar::AStarConfig;
use pathforge::budget::BudgetedPathfinder;
use pathforge::graphs::grid2d::{Grid2D, GridPos, DiagonalMode};
use pathforge::heuristics::Diagonal;

let grid = Grid2D::new(2048, 2048, DiagonalMode::Always);
let start = GridPos { x: 0, y: 0 };
let goal = GridPos { x: 2000, y: 2000 };
let heuristic = Diagonal::default();
let mut pathfinder = BudgetedPathfinder::new(AStarConfig::default());
pathfinder.start(start, goal, &heuristic);

// Game loop with 0.5 ms budget
for _frame in 0..100 {
    let done = pathfinder.step(&grid, &heuristic, Duration::from_micros(500));
    if let Some(partial) = pathfinder.partial_result() {
        // Use partial.path for early movement if you want
    }
    if done {
        let result = pathfinder.take_result().unwrap();
        println!("Found path: {} nodes, cost {}", result.path.len(), result.cost);
        break;
    }
}
```

## Algorithms: when to use what
- **A\***: default choice; pair with Diagonal heuristic on grids.
- **JPS**: uniform-cost grids only; empty or lightly obstructed maps see 10x+ gains.
- **Flow field**: RTS swarms; compute once, agents query direction in O(1).
- **Dijkstra**: use A* with `Zero` heuristic for weighted graphs needing uninformed search.

## Benchmarks (cargo bench, release)

### A*
| Scenario | Grid | Nodes Expanded | Time |
|----------|------|----------------|------|
| Empty diagonal | 100x100 | 97 | ~81 µs |
| Empty diagonal | 128x128 | 100 | ~82 µs |
| Empty diagonal | 1024x1024 | 1021 | ~865 µs |
| Maze | 64x64 | 1327 | ~512 µs |

### JPS vs A* (uniform grids)
| Scenario | A* Nodes | JPS Nodes | A* Time | JPS Time | Speedup |
|----------|----------|-----------|---------|----------|---------|
| Empty 128x128 | 100 | 1 | ~85 µs | ~41 µs | ~2x |
| Maze 64x64 | 1327 | 31 | ~512 µs | ~41 µs | ~12x |

### Flow field compute (one-time)
| Grid | Time |
|------|------|
| 1024x1024 | ~119 ms |
| 2048x2048 | ~532 ms |

## More examples
- `examples/frame_budget.rs`: hero example for frame budgeting with partial progress logs.
- `examples/flowfield_demo.rs`: small visualization plus 1024/2048 timing and bilinear sampling. Run with `cargo run --release --example flowfield_demo`.
- `examples/path_cache.rs`: cache hits on repeated queries.
- `examples/basic_grid.rs`, `examples/weighted_terrain.rs`, `examples/dynamic_obstacles.rs`: standard patterns (add your own graph types via the `Graph` trait).

## Custom graphs
Implement `Graph` to plug in hex grids, navmeshes, or your own world:

```rust
use pathforge::traits::Graph;

impl Graph for MyGraph {
    type Node = MyNode;
    fn is_passable(&self, node: &Self::Node) -> bool { /* ... */ }
    fn neighbors<F>(&self, node: &Self::Node, mut visit: F)
    where F: FnMut(Self::Node, f32) {
        // visit(neighbor, cost);
    }
}
```

## Feature flags
- `parallel`: enable rayon-backed parallel preprocessing where applicable (default).
- `serde`: planned for serializing grids/flow fields.

## Roadmap
- NavMesh support
- HPA* for very large worlds
- 3D grids and Theta*
- serde support for graphs/flow fields

## License
MIT OR Apache-2.0, at your option.

## Contributing
Issues and PRs welcome. Please run tests/benches in release mode when reporting performance numbers.

---
Note: portions of this codebase and documentation were produced with AI assistance.***
