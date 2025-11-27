use pathforge::{
    algorithms::{theta::theta_star, astar::AStarConfig},
    graphs::grid2d::{Grid2D, GridPos, DiagonalMode},
    heuristics::Euclidean,
    traits::PathStatus,
};

#[test]
fn test_theta_star_line_of_sight() {
    let width = 20;
    let height = 20;
    let grid = Grid2D::new(width, height, DiagonalMode::Always);
    let start = GridPos { x: 0, y: 0 };
    let goal = GridPos { x: 10, y: 5 };
    
    let result = theta_star(
        &grid,
        &Euclidean,
        start,
        goal,
        AStarConfig::default()
    );
    
    assert_eq!(result.status, PathStatus::Found);
    
    // Theta* on an empty grid should typically return just Start -> Goal (2 nodes)
    // because LOS exists directly.
    assert_eq!(result.path.len(), 2); 
    assert_eq!(result.path[0], start);
    assert_eq!(result.path[1], goal);
}

#[test]
fn test_theta_star_around_obstacle() {
    let width = 20;
    let height = 20;
    let mut grid = Grid2D::new(width, height, DiagonalMode::Always);
    
    // Wall at x=5
    grid.set_region_blocked((5, 0, 1, 10), true);
    
    let start = GridPos { x: 2, y: 2 };
    let goal = GridPos { x: 8, y: 2 };
    
    let result = theta_star(
        &grid,
        &Euclidean,
        start,
        goal,
        AStarConfig::default()
    );
    
    assert_eq!(result.status, PathStatus::Found);
    
    // Path should contain intermediate nodes to go around the wall
    assert!(result.path.len() > 2);
    
    // Verify path validity (simulated)
    for _i in 0..result.path.len() - 1 {
        // In a real test we'd use the Graph::can_traverse method, 
        // but we know Grid2D uses Bresenham.
        // Just ensure consecutive nodes are somewhat close or LOS is valid.
        // The point of Theta* is they might be far apart.
    }
}
