use std::time::Duration;

use pathforge::algorithms::astar::AStarConfig;
use pathforge::budget::BudgetedPathfinder;
use pathforge::graphs::grid2d::{DiagonalMode, Grid2D, GridPos};
use pathforge::heuristics::Diagonal;

fn main() {
    let width = 2048;
    let height = 2048;
    let grid = Grid2D::new(width, height, DiagonalMode::Always);

    let start = GridPos { x: 0, y: 0 };
    let goal = GridPos { x: 2000, y: 2000 };

    let mut pathfinder = BudgetedPathfinder::new(AStarConfig::default());
    let heuristic = Diagonal::default();
    pathfinder.start(start, goal, &heuristic);

    // Simulate a game/render loop with a 0.5 ms frame budget for pathfinding work.
    for frame in 0..100 {
        let done = pathfinder.step(&grid, &heuristic, Duration::from_micros(500));

        if let Some(partial) = pathfinder.partial_result() {
            println!(
                "frame {frame:02}: expanded {} nodes, partial path length {} (status {:?})",
                partial.nodes_expanded,
                partial.path.len(),
                partial.status
            );
        }

        if done {
            if let Some(result) = pathfinder.take_result() {
                println!(
                    "complete on frame {frame:02}: status {:?}, nodes {}, path length {}, cost {}",
                    result.status,
                    result.nodes_expanded,
                    result.path.len(),
                    result.cost
                );
            }
            break;
        }
    }
}
