use criterion::{black_box, criterion_group, criterion_main, Criterion};
use pathforge::algorithms::astar::{astar, AStarConfig};
use pathforge::algorithms::jps::jps;
use pathforge::graphs::grid2d::{Grid2D, GridPos, DiagonalMode};
use pathforge::heuristics::Diagonal;

fn bench_jps_vs_astar(c: &mut Criterion) {
    let width = 128;
    let height = 128;
    let grid = Grid2D::new(width, height, DiagonalMode::Always);
    let heuristic = Diagonal::default();
    let start = GridPos { x: 10, y: 10 };
    let goal = GridPos { x: 110, y: 110 };

    let mut group = c.benchmark_group("jps_vs_astar_empty");
    
    group.bench_function("astar", |b| {
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
    
    group.bench_function("jps", |b| {
        b.iter(|| {
            jps(
                black_box(&grid),
                black_box(&heuristic),
                black_box(start),
                black_box(goal),
                black_box(AStarConfig::default()),
            )
        })
    });
    
    group.finish();
    
    let mut grid_maze = Grid2D::new(width, height, DiagonalMode::Always);
    for x in (4..width).step_by(4) {
        grid_maze.set_region_blocked((x, 0, 1, height - 10), true);
    }
    
    let mut group = c.benchmark_group("jps_vs_astar_maze");
    
    group.bench_function("astar", |b| {
        b.iter(|| {
            astar(
                black_box(&grid_maze),
                black_box(&heuristic),
                black_box(start),
                black_box(goal),
                black_box(AStarConfig::default()),
            )
        })
    });
    
    group.bench_function("jps", |b| {
        b.iter(|| {
            jps(
                black_box(&grid_maze),
                black_box(&heuristic),
                black_box(start),
                black_box(goal),
                black_box(AStarConfig::default()),
            )
        })
    });
    
    group.finish();
}

criterion_group!(benches, bench_jps_vs_astar);
criterion_main!(benches);
