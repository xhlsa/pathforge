//! Flow field pathfinding demo.
//!
//! Run with: `cargo run --release --example flowfield_demo`
//! (Release mode is ~25x faster for compute-heavy operations)

use std::time::Instant;

use pathforge::flowfield::{Direction, FlowField};
use pathforge::graphs::grid2d::{DiagonalMode, Grid2D, GridPos};

fn dir_to_char(d: Direction) -> char {
    match d {
        Direction::None => '.',
        Direction::N => '↑',
        Direction::NE => '↗',
        Direction::E => '→',
        Direction::SE => '↘',
        Direction::S => '↓',
        Direction::SW => '↙',
        Direction::W => '←',
        Direction::NW => '↖',
    }
}

fn main() {
    // Small demo: 8x8 grid, goal in the corner. Print local flow directions.
    let mut small = Grid2D::new(8, 8, DiagonalMode::Always);
    small.set_blocked(3, 3, true);
    small.set_blocked(3, 4, true);
    let goal = GridPos { x: 7, y: 7 };
    let ff_small = FlowField::compute(&small, goal);
    println!("Flow directions (8x8), goal at (7,7):");
    for y in 0..8 {
        for x in 0..8 {
            let d = ff_small.get_direction(GridPos { x, y });
            print!("{} ", dir_to_char(d));
        }
        println!();
    }

    // Timing demo: 1024x1024 uniform grid.
    let big_size = 1024usize;
    let big = Grid2D::new(big_size, big_size, DiagonalMode::Always);
    let goal_big = GridPos { x: 700, y: 700 };
    let start = Instant::now();
    let ff_big = FlowField::compute(&big, goal_big);
    let elapsed = start.elapsed();
    println!(
        "1024x1024 flow field computed in {:?}, sample direction at (0,0) = {:?}, cost_to_goal = {:.2}",
        elapsed,
        ff_big.get_direction(GridPos { x: 0, y: 0 }),
        ff_big.get_cost_to_goal(GridPos { x: 0, y: 0 }),
    );

    // Sample bilinear steering near (10.3, 10.8)
    let (vx, vy) = ff_big.sample_bilinear(10.3, 10.8);
    println!("Sample bilinear at (10.3,10.8): ({:.3}, {:.3})", vx, vy);

    // Larger stress: 2048x2048
    let big2_size = 2048usize;
    let big2 = Grid2D::new(big2_size, big2_size, DiagonalMode::Always);
    let goal_big2 = GridPos { x: 1500, y: 1500 };
    let start_big2 = Instant::now();
    let ff_big2 = FlowField::compute(&big2, goal_big2);
    let elapsed_big2 = start_big2.elapsed();
    println!(
        "2048x2048 flow field computed in {:?}, sample direction at (0,0) = {:?}, cost_to_goal = {:.2}",
        elapsed_big2,
        ff_big2.get_direction(GridPos { x: 0, y: 0 }),
        ff_big2.get_cost_to_goal(GridPos { x: 0, y: 0 }),
    );
}
