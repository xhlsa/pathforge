# Pathforge API Reference

This document serves as a manual reference for the `pathforge` library.

## Overview

`pathforge` is a high-performance pathfinding library for Rust. It provides generic algorithms like A* and Theta*, as well as specialized high-speed algorithms like Jump Point Search (JPS) and Flow Fields for grid-based maps.

## Core Traits (`pathforge::traits`)

The library is built around a few core traits that allow you to use the algorithms with any graph structure.

### `Graph`

The `Graph` trait defines the structure of your world. You must implement this for your own map types if you aren't using the built-in `Grid2D`.

```rust
pub trait Graph {
    // The type used to identify nodes (e.g., struct, integer ID, coordinate).
    // Must implement Copy, Eq, Hash, Debug.
    type Node: Copy + Eq + Hash + Debug;

    // Check if a node can be entered/traversed.
    fn is_passable(&self, node: &Self::Node) -> bool;

    // Find neighbors of a node.
    // The closure `visit` should be called for each neighbor with (node, edge_cost).
    fn neighbors<F>(&self, node: &Self::Node, visit: F)
    where F: FnMut(Self::Node, f32);
    
    // Optional: Optimized check for direct line-of-sight (for Theta*).
    // Defaults to step-by-step raycasting if not overridden.
    fn line_of_sight(&self, start: &Self::Node, end: &Self::Node) -> bool { ... }
}
```

### `Heuristic`

The `Heuristic` trait defines how to estimate the cost between two nodes.

```rust
pub trait Heuristic<Node> {
    fn estimate(&self, from: &Node, to: &Node) -> f32;
}
```

## Algorithms (`pathforge::algorithms`)

### A* (`astar::astar`)

The standard pathfinding algorithm. Works with any type implementing `Graph`.

```rust
use pathforge::algorithms::astar::{astar, AStarConfig};

let config = AStarConfig {
    tie_breaking: true, // Adds slight deterministic noise to costs to break symmetries
    ..Default::default()
};

let result = astar(
    &graph,
    start_node,
    goal_node,
    &heuristic,
    config
);
```

**Returns:** `Option<PathResult>` containing the path (Vec of nodes) and total cost.

### Theta* (`theta::theta_star`)

Any-angle pathfinding. Like A*, but nodes can connect to any other visible node, not just immediate neighbors. Produces smoother paths than A*.

```rust
use pathforge::algorithms::theta::theta_star;

let result = theta_star(
    &graph,
    start_node,
    goal_node,
    &heuristic,
    Default::default() // Uses AStarConfig
);
```

### Jump Point Search (`jps::jps`)

**Specialized:** Only works with `Grid2D`.
Significantly faster than A* on uniform-cost grids by "jumping" over empty space.

```rust
use pathforge::algorithms::jps::jps;

let result = jps(
    &grid, // Must be pathforge::graphs::grid2d::Grid2D
    start_pos,
    goal_pos,
    &heuristic
);
```

### Flow Fields (`flowfield::FlowField`)

**Specialized:** Only works with `Grid2D`.
Best for moving many agents to a single target.

```rust
use pathforge::algorithms::flowfield::FlowField;

// 1. Compute the field once (expensive)
let flow_field = FlowField::compute(&grid, target_pos);

// 2. Query direction for any agent (O(1), very fast)
let direction = flow_field.get_direction(agent_pos);
```

## Graphs (`pathforge::graphs`)

### `Grid2D`

A concrete, feature-rich 2D grid implementation.

```rust
use pathforge::graphs::grid2d::{Grid2D, DiagonalMode};

// Create a 100x100 grid
// DiagonalMode::Always - Allow diagonal movement
// DiagonalMode::NoCutCorners - Allow diagonal only if cardinal neighbors are free
// DiagonalMode::Never - Manhattan movement only
let mut grid = Grid2D::new(100, 100, DiagonalMode::Always);

// Set terrain costs
grid.set_cost(10, 10, 5.0); // High cost (mud/water)

// Block cells
grid.set_blocked(5, 5, true); // Wall
```

## Heuristics (`pathforge::heuristics`)

Standard heuristic implementations.

*   `Manhattan`: dx + dy. Best for 4-way movement.
*   `Euclidean`: sqrt(dx*dx + dy*dy). Best for any-angle or real-world distance.
*   `Diagonal`: Approx distance on an 8-way grid (moves diagonally then straight).
*   `Zero`: Returns 0. Turns A* into Dijkstra's Algorithm (guarantees shortest path but explores more nodes).
