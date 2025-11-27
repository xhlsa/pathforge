use pathforge::{
    budget::BudgetedPathfinder,
    algorithms::astar::AStarConfig,
    graphs::grid2d::{Grid2D, GridPos, DiagonalMode},
    heuristics::Diagonal,
};
use std::time::{Duration, Instant};

fn main() {
    let width = 500;
    let height = 500;
    let mut grid = Grid2D::new(width, height, DiagonalMode::Always);
    
    // Maze
    for x in (10..490).step_by(10) {
        grid.set_region_blocked((x, 0, 2, 400), true);
    }
    
    let start = GridPos { x: 5, y: 5 };
    let goal = GridPos { x: 495, y: 495 };
    
    let config = AStarConfig::default();
    let mut pathfinder = BudgetedPathfinder::new(config);
    let heuristic = Diagonal::default();
    
    println!("Starting budgeted pathfinding...");
    pathfinder.start(start, goal, &heuristic);
    
    let mut frame = 0;
    let budget = Duration::from_micros(500); // 0.5 ms per frame
    let start_time = Instant::now();
    
    loop {
        frame += 1;
        let complete = pathfinder.step(&grid, &heuristic, budget);
        
        if complete {
            println!("Pathfinding complete on frame {}!", frame);
            break;
        }
        
        // Simulate other frame work
        // thread::sleep(Duration::from_millis(16));
    }
    
    let total_duration = start_time.elapsed();
    println!("Total wall time: {:.2?}", total_duration);
    
    if let Some(result) = pathfinder.take_result() {
        println!("Status: {:?}, Cost: {:.2}, Nodes: {}", result.status, result.cost, result.nodes_expanded);
    }
}
