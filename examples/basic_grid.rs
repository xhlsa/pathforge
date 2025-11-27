use pathforge::{
    algorithms::astar::{astar, AStarConfig},
    graphs::grid2d::{Grid2D, GridPos, DiagonalMode},
    heuristics::Diagonal,
    traits::PathStatus,
};

fn main() {
    let width = 20;
    let height = 20;
    let mut grid = Grid2D::new(width, height, DiagonalMode::Always);
    
    // Add a wall
    // Wall from (5,0) to (5, 15)
    grid.set_region_blocked((5, 0, 1, 15), true);
    
    let start = GridPos { x: 2, y: 10 };
    let goal = GridPos { x: 15, y: 10 };
    
    println!("Finding path from {:?} to {:?}...", start, goal);
    
    let config = AStarConfig::default();
    let heuristic = Diagonal::default();
    
    let result = astar(&grid, &heuristic, start, goal, config);
    
    match result.status {
        PathStatus::Found => {
            println!("Path found! Cost: {:.2}, Nodes expanded: {}", result.cost, result.nodes_expanded);
            println!("Path length: {}", result.path.len());
            
            // Visualize
            for y in 0..height {
                for x in 0..width {
                    let pos = GridPos { x: x as i32, y: y as i32 };
                    if pos == start {
                        print!("S");
                    } else if pos == goal {
                        print!("G");
                    } else if result.path.contains(&pos) {
                        print!("*");
                    } else if grid.is_blocked(x as i32, y as i32) {
                        print!("#");
                    } else {
                        print!(".");
                    }
                }
                println!();
            }
        }
        _ => println!("Path not found or incomplete: {:?}", result.status),
    }
}
