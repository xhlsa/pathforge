use pathforge::{
    graphs::{grid2d::{Grid2D, GridPos, DiagonalMode}, hierarchical::HierarchicalGrid},
    traits::PathStatus,
};
use std::time::Instant;

fn main() {
    let width = 100;
    let height = 100;
    let mut grid = Grid2D::new(width, height, DiagonalMode::Always);
    
    // Create a "room" structure to make HPA* interesting
    // Grid of rooms, connected by gaps
    for x in (0..width).step_by(10) {
        grid.set_region_blocked((x, 0, 1, height), true);
    }
    for y in (0..height).step_by(10) {
        grid.set_region_blocked((0, y, width, 1), true);
    }
    
    // Open gaps
    for x in (0..width).step_by(10) {
        for y in (5..height).step_by(10) {
             grid.set_blocked(x, y, false); // Horizontal gaps
        }
    }
    for y in (0..height).step_by(10) {
        for x in (5..width).step_by(10) {
             grid.set_blocked(x, y, false); // Vertical gaps
        }
    }

    println!("Preprocessing Hierarchical Grid...");
    let start_pre = Instant::now();
    let h_grid = HierarchicalGrid::new(grid, 10); // 10x10 clusters match rooms
    println!("Preprocessing took {:.2?}", start_pre.elapsed());
    println!("Created {} abstract nodes.", h_grid.nodes.len());
    
    let start = GridPos { x: 2, y: 2 };
    let goal = GridPos { x: 92, y: 92 };
    
    println!("Finding path...");
    let start_search = Instant::now();
    let result = h_grid.find_path(start, goal);
    let duration = start_search.elapsed();
    
    match result.status {
        PathStatus::Found => {
            println!("Path found in {:.2?}!", duration);
            println!("Path length: {}, Cost: {:.2}", result.path.len(), result.cost);
            println!("Abstract nodes expanded: {}", result.nodes_expanded);
        }
        _ => println!("Path not found: {:?}", result.status),
    }
}
