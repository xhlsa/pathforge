use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::graphs::grid2d::{DiagonalMode, Grid2D, GridPos};
use crate::traits::Graph;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Direction {
    #[default]
    None,
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW,
}

impl Direction {
    fn to_vec2(self) -> (f32, f32) {
        match self {
            Direction::None => (0.0, 0.0),
            Direction::N => (0.0, -1.0),
            Direction::NE => (1.0, -1.0),
            Direction::E => (1.0, 0.0),
            Direction::SE => (1.0, 1.0),
            Direction::S => (0.0, 1.0),
            Direction::SW => (-1.0, 1.0),
            Direction::W => (-1.0, 0.0),
            Direction::NW => (-1.0, -1.0),
        }
    }
}

#[derive(Clone)]
pub struct FlowField {
    pub width: usize,
    pub height: usize,
    pub integration: Vec<f32>, // cost-to-goal
    pub flow: Vec<Direction>,  // best step toward goal
}

impl FlowField {
    pub fn compute(grid: &Grid2D, goal: GridPos) -> Self {
        let width = grid.width;
        let height = grid.height;
        let len = width * height;
        let mut integration = vec![f32::INFINITY; len];
        let mut flow = vec![Direction::None; len];
        let mut visited = vec![false; len];

        if goal.x < 0
            || goal.y < 0
            || goal.x as usize >= width
            || goal.y as usize >= height
            || grid.is_blocked(goal.x, goal.y)
        {
            return Self {
                width,
                height,
                integration,
                flow,
            };
        }

        let mut frontier = BinaryHeap::new();
        let goal_idx = Self::idx(width, goal.x as usize, goal.y as usize);
        integration[goal_idx] = 0.0;
        frontier.push(State {
            cost: 0.0,
            pos: goal,
        });

        // Dijkstra from goal outward (integration field)
        while let Some(State { cost, pos }) = frontier.pop() {
            let idx = Self::idx(width, pos.x as usize, pos.y as usize);
            if visited[idx] || cost > integration[idx] {
                continue;
            }
            visited[idx] = true;

            grid.neighbors(&pos, |n, edge_cost| {
                if grid.is_blocked(n.x, n.y) {
                    return;
                }
                let next_cost = cost + edge_cost;
                if n.x < 0
                    || n.y < 0
                    || n.x as usize >= width
                    || n.y as usize >= height
                {
                    return;
                }
                let n_idx = Self::idx(width, n.x as usize, n.y as usize);
                if next_cost < integration[n_idx] {
                    integration[n_idx] = next_cost;
                    frontier.push(State {
                        cost: next_cost,
                        pos: n,
                    });
                }
            });
        }

        // Flow pass: choose neighbor with lowest integration value
        for y in 0..height {
            for x in 0..width {
                let idx = Self::idx(width, x, y);
                if integration[idx].is_infinite() || grid.is_blocked(x as i32, y as i32) {
                    continue;
                }
                let mut best_dir = Direction::None;
                let mut best_cost = integration[idx];
                for &(dx, dy, dir) in Self::neighbor_dirs(grid.diagonal_movement) {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx < 0 || ny < 0 || nx as usize >= width || ny as usize >= height {
                        continue;
                    }
                    if grid.is_blocked(nx, ny) {
                        continue;
                    }
                    let n_idx = Self::idx(width, nx as usize, ny as usize);
                    let n_cost = integration[n_idx];
                    if n_cost < best_cost {
                        best_cost = n_cost;
                        best_dir = dir;
                    }
                }
                flow[idx] = best_dir;
            }
        }

        Self {
            width,
            height,
            integration,
            flow,
        }
    }

    #[inline]
    pub fn get_direction(&self, pos: GridPos) -> Direction {
        if pos.x < 0
            || pos.y < 0
            || pos.x as usize >= self.width
            || pos.y as usize >= self.height
        {
            return Direction::None;
        }
        let idx = Self::idx(self.width, pos.x as usize, pos.y as usize);
        self.flow[idx]
    }

    #[inline]
    pub fn get_cost_to_goal(&self, pos: GridPos) -> f32 {
        if pos.x < 0
            || pos.y < 0
            || pos.x as usize >= self.width
            || pos.y as usize >= self.height
        {
            return f32::INFINITY;
        }
        let idx = Self::idx(self.width, pos.x as usize, pos.y as usize);
        self.integration[idx]
    }

    /// Returns a smoothed flow vector using bilinear sampling of the 4 surrounding cells.
    pub fn sample_bilinear(&self, x: f32, y: f32) -> (f32, f32) {
        if x < 0.0 || y < 0.0 {
            return (0.0, 0.0);
        }
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;
        if x0 < 0
            || y0 < 0
            || x1 as usize >= self.width
            || y1 as usize >= self.height
        {
            return (0.0, 0.0);
        }

        let fx = x - x0 as f32;
        let fy = y - y0 as f32;

        let v00 = self.get_direction(GridPos { x: x0, y: y0 }).to_vec2();
        let v10 = self.get_direction(GridPos { x: x1, y: y0 }).to_vec2();
        let v01 = self.get_direction(GridPos { x: x0, y: y1 }).to_vec2();
        let v11 = self.get_direction(GridPos { x: x1, y: y1 }).to_vec2();

        let vx0 = lerp(v00.0, v10.0, fx);
        let vy0 = lerp(v00.1, v10.1, fx);
        let vx1 = lerp(v01.0, v11.0, fx);
        let vy1 = lerp(v01.1, v11.1, fx);

        (lerp(vx0, vx1, fy), lerp(vy0, vy1, fy))
    }

    #[inline]
    fn idx(width: usize, x: usize, y: usize) -> usize {
        y * width + x
    }

    fn neighbor_dirs(diag: DiagonalMode) -> &'static [(i32, i32, Direction)] {
        match diag {
            DiagonalMode::Never => &[
                (0, -1, Direction::N),
                (1, 0, Direction::E),
                (0, 1, Direction::S),
                (-1, 0, Direction::W),
            ],
            _ => &[
                (0, -1, Direction::N),
                (1, -1, Direction::NE),
                (1, 0, Direction::E),
                (1, 1, Direction::SE),
                (0, 1, Direction::S),
                (-1, 1, Direction::SW),
                (-1, 0, Direction::W),
                (-1, -1, Direction::NW),
            ],
        }
    }
}

#[derive(Copy, Clone)]
struct State {
    cost: f32,
    pos: GridPos,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for min-heap behavior
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graphs::grid2d::DiagonalMode;

    #[test]
    fn simple_cardinal_field_points_to_goal() {
        let mut grid = Grid2D::new(3, 1, DiagonalMode::Never);
        grid.set_blocked(1, 0, true);
        let ff = FlowField::compute(&grid, GridPos { x: 2, y: 0 });
        assert_eq!(ff.get_direction(GridPos { x: 0, y: 0 }), Direction::None);
        assert_eq!(ff.get_cost_to_goal(GridPos { x: 0, y: 0 }), f32::INFINITY);
        assert_eq!(ff.get_direction(GridPos { x: 2, y: 0 }), Direction::None);
        assert_eq!(ff.get_cost_to_goal(GridPos { x: 2, y: 0 }), 0.0);
    }

    #[test]
    fn diagonal_field_prefers_shortcut() {
        let grid = Grid2D::new(3, 3, DiagonalMode::Always);
        let ff = FlowField::compute(&grid, GridPos { x: 2, y: 2 });
        assert_eq!(ff.get_direction(GridPos { x: 0, y: 0 }), Direction::SE);
    }
}
