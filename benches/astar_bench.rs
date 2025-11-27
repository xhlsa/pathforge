use criterion::{black_box, criterion_group, criterion_main, Criterion};
use pathforge::algorithms::astar::{astar, AStarConfig};
use pathforge::graphs::grid2d::{Grid2D, GridPos, DiagonalMode};
use pathforge::heuristics::Diagonal;

fn bench_astar_empty(c: &mut Criterion) {
    let width = 128;
    let height = 128;
    let grid = Grid2D::new(width, height, DiagonalMode::Always);
    let heuristic = Diagonal::default();
    let start = GridPos { x: 10, y: 10 };
    let goal = GridPos { x: 110, y: 110 };

    let sample = astar(
        &grid,
        &heuristic,
        start,
        goal,
        AStarConfig::default(),
    );
    println!("nodes_expanded (astar_empty_128x128): {}", sample.nodes_expanded);

    c.bench_function("astar_empty_128x128", |b| {
        b.iter(|| {
            astar(
                black_box(&grid),
                black_box(&heuristic),
                black_box(start),
                black_box(goal),
                black_box(AStarConfig::default()),
            )
        })
    });
}

fn bench_astar_empty_100(c: &mut Criterion) {
    let width = 100;
    let height = 100;
    let grid = Grid2D::new(width, height, DiagonalMode::Always);
    let heuristic = Diagonal::default();
    let start = GridPos { x: 1, y: 1 };
    let goal = GridPos { x: 98, y: 98 };

    let sample = astar(
        &grid,
        &heuristic,
        start,
        goal,
        AStarConfig::default(),
    );
    println!("nodes_expanded (astar_empty_100x100): {}", sample.nodes_expanded);

    c.bench_function("astar_empty_100x100", |b| {
        b.iter(|| {
            astar(
                black_box(&grid),
                black_box(&heuristic),
                black_box(start),
                black_box(goal),
                black_box(AStarConfig::default()),
            )
        })
    });
}

fn bench_astar_maze(c: &mut Criterion) {
    let width = 64;
    let height = 64;
    let mut grid = Grid2D::new(width, height, DiagonalMode::Always);
    // Simple maze: vertical walls every 4 units with gaps
    for x in (4..width).step_by(4) {
        grid.set_region_blocked((x, 0, 1, height - 5), true);
    }
    
    let heuristic = Diagonal::default();
    let start = GridPos { x: 1, y: 1 };
    let goal = GridPos { x: 60, y: 60 };

    let sample = astar(
        &grid,
        &heuristic,
        start,
        goal,
        AStarConfig::default(),
    );
    println!("nodes_expanded (astar_maze_64x64): {}", sample.nodes_expanded);

    c.bench_function("astar_maze_64x64", |b| {
        b.iter(|| {
            astar(
                black_box(&grid),
                black_box(&heuristic),
                black_box(start),
                black_box(goal),
                black_box(AStarConfig::default()),
            )
        })
    });
}

fn bench_astar_empty_1024(c: &mut Criterion) {
    let width = 1024;
    let height = 1024;
    let grid = Grid2D::new(width, height, DiagonalMode::Always);
    let heuristic = Diagonal::default();
    let start = GridPos { x: 1, y: 1 };
    let goal = GridPos { x: 1022, y: 1022 };

    let sample = astar(
        &grid,
        &heuristic,
        start,
        goal,
        AStarConfig::default(),
    );
    println!("nodes_expanded (astar_empty_1024x1024): {}", sample.nodes_expanded);

    c.bench_function("astar_empty_1024x1024", |b| {
        b.iter(|| {
            astar(
                black_box(&grid),
                black_box(&heuristic),
                black_box(start),
                black_box(goal),
                black_box(AStarConfig::default()),
            )
        })
    });
}

criterion_group!(
    benches,
    bench_astar_empty,
    bench_astar_empty_100,
    bench_astar_empty_1024,
    bench_astar_maze
);
criterion_main!(benches);
