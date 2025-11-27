use std::time::Duration;

use pathforge::algorithms::astar::AStarConfig;
use pathforge::cache::{astar_with_cache, PathCache};
use pathforge::graphs::grid2d::{DiagonalMode, Grid2D, GridPos};
use pathforge::heuristics::Diagonal;

fn main() {
    let grid = Grid2D::new(32, 32, DiagonalMode::Always);
    let mut cache = PathCache::new(8, Duration::from_secs(10));

    let start = GridPos { x: 0, y: 0 };
    let goal = GridPos { x: 30, y: 30 };

    // First query: cache miss, runs A*
    let res1 = astar_with_cache(
        &grid,
        &Diagonal::default(),
        start,
        goal,
        AStarConfig::default(),
        &mut cache,
    );
    println!(
        "first query: status {:?}, nodes {}, cache size {}",
        res1.status,
        res1.nodes_expanded,
        cache.len()
    );

    // Second query: same endpoints, served from cache
    let res2 = astar_with_cache(
        &grid,
        &Diagonal::default(),
        GridPos { x: 0, y: 0 },
        GridPos { x: 30, y: 30 },
        AStarConfig::default(),
        &mut cache,
    );
    println!(
        "second query (cached): status {:?}, nodes {}, cache size {}",
        res2.status,
        res2.nodes_expanded,
        cache.len()
    );
}
