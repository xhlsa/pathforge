use pathforge::{
    algorithms::{astar::{astar, AStarConfig}, jps::jps},
    graphs::grid2d::{Grid2D, GridPos, DiagonalMode},
    heuristics::Diagonal,
    traits::PathStatus,
};

#[test]
fn test_astar_vs_jps_correctness() {
    let width = 50;
    let height = 50;
    let mut grid = Grid2D::new(width, height, DiagonalMode::Always);
    
    // Add random obstacles deterministic
    for i in 0..(width*height/3) {
        let x = (i * 123 + 5) % width;
        let y = (i * 456 + 7) % height;
        grid.set_blocked(x, y, true);
    }
    
    let heuristic = Diagonal::default();
    let config = AStarConfig::default();
    
    let start = GridPos { x: 0, y: 0 };
    let goal = GridPos { x: 49, y: 49 };
    
    // Ensure start/goal passable
    grid.set_blocked(0, 0, false);
    grid.set_blocked(49, 49, false);
    
    let res_astar = astar(&grid, &heuristic, start, goal, config);
    let res_jps = jps(&grid, &heuristic, start, goal, config);
    
    assert_eq!(res_astar.status, res_jps.status);
    
    if res_astar.status == PathStatus::Found {
        // JPS cost should be optimal (same as A*)
        // Floating point comparison
        assert!((res_astar.cost - res_jps.cost).abs() < 1e-4, "Costs differ: A*={}, JPS={}", res_astar.cost, res_jps.cost);
    }
}
