use pathforge::{
    algorithms::{astar::AStarConfig, parallel::find_paths_parallel},
    graphs::grid2d::{Grid2D, GridPos, DiagonalMode},
    heuristics::Diagonal,
};
use std::time::Instant;

fn main() {
    let width = 100;
    let height = 100;
    let mut grid = Grid2D::new(width, height, DiagonalMode::Always);
    
    // Add random obstacles
    // Simple pseudo-random generator
    for i in 0..(width*height/4) {
        let x = (i * 37) % width;
        let y = (i * 113) % height;
        grid.set_blocked(x, y, true);
    }
    
    let heuristic = Diagonal::default();
    let config = AStarConfig::default();
    
    // Generate 1000 queries
    let mut queries = Vec::new();
    for i in 0..1000 {
        let start = GridPos { x: (i % 10) as i32, y: (i % 10) as i32 };
        let goal = GridPos { x: (width - 1 - (i % 10)) as i32, y: (height - 1 - (i % 10)) as i32 };
        queries.push((start, goal));
    }
    
    println!("Solving {} paths in parallel...", queries.len());
    let start_time = Instant::now();
    let results = find_paths_parallel(&grid, &heuristic, &queries, config);
    let duration = start_time.elapsed();
    
    let found_count = results.iter().filter(|r| r.status == pathforge::traits::PathStatus::Found).count();
    
    println!("Solved in {:.2?}. Found: {}/{}", duration, found_count, queries.len());
}
