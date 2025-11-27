use crate::graphs::grid2d::{Grid2D, GridPos, DiagonalMode};
use crate::traits::{PathResult, PathStatus, Heuristic};
use crate::algorithms::astar::{AStarConfig, TieBreaking};
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;
use std::time::Instant;

#[derive(Clone, Copy)]
struct State {
    node: GridPos,
    cost: f32,
    g_score: f32,
    tie_breaker: f32,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.tie_breaker == other.tie_breaker
    }
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.cost < other.cost {
            return Ordering::Greater;
        } else if self.cost > other.cost {
            return Ordering::Less;
        }
        if self.tie_breaker > other.tie_breaker {
            return Ordering::Greater;
        } else if self.tie_breaker < other.tie_breaker {
            return Ordering::Less;
        }
        Ordering::Equal
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn jps<H>(
    grid: &Grid2D,
    heuristic: &H,
    start: GridPos,
    goal: GridPos,
    config: AStarConfig,
) -> PathResult<GridPos>
where
    H: Heuristic<GridPos>,
{
    let start_time = Instant::now();
    let mut open_set = BinaryHeap::new();
    let mut g_scores = HashMap::new();
    let mut came_from = HashMap::new();
    
    g_scores.insert(start, 0.0);
    
    open_set.push(State {
        node: start,
        cost: heuristic.estimate(&start, &goal),
        g_score: 0.0,
        tie_breaker: 0.0,
    });
    
    let mut nodes_expanded = 0;
    let mut iterations = 0;
    
    while let Some(State { node: current, cost: _, g_score: current_g, tie_breaker: _ }) = open_set.pop() {
        iterations += 1;
        
        if let Some(max_iter) = config.max_iterations {
            if iterations > max_iter {
                 return reconstruct_path(current, &came_from, current_g, nodes_expanded, PathStatus::PartialMaxIter);
            }
        }
        if let Some(timeout) = config.timeout {
            if start_time.elapsed() > timeout {
                return reconstruct_path(current, &came_from, current_g, nodes_expanded, PathStatus::PartialTimeout);
            }
        }
        
        if current == goal {
            return reconstruct_path(current, &came_from, current_g, nodes_expanded, PathStatus::Found);
        }
        
        if let Some(&best_g) = g_scores.get(&current) {
            if current_g > best_g {
                continue;
            }
        }
        
        nodes_expanded += 1;
        
        let parent = came_from.get(&current).cloned();
        
        let successors = find_successors(grid, current, parent, goal);
        
        for neighbor in successors {
            let dist = distance(current, neighbor);
            let tentative_g = current_g + dist;
            
            if let Some(&existing_g) = g_scores.get(&neighbor) {
                if tentative_g >= existing_g {
                    continue;
                }
            }
            
            came_from.insert(neighbor, current);
            g_scores.insert(neighbor, tentative_g);
            
            let h = heuristic.estimate(&neighbor, &goal);
            let f = tentative_g + h;
            
            let tb = match config.tie_breaking {
                TieBreaking::None => 0.0,
                TieBreaking::PreferHigherG => tentative_g,
                TieBreaking::PreferLowerG => -tentative_g,
                TieBreaking::CrossProduct => 0.0,
            };
            
            open_set.push(State {
                node: neighbor,
                cost: f,
                g_score: tentative_g,
                tie_breaker: tb,
            });
        }
    }
    
    PathResult {
        path: vec![],
        cost: 0.0,
        nodes_expanded,
        status: PathStatus::NotFound,
    }
}

fn distance(a: GridPos, b: GridPos) -> f32 {
    let dx = (a.x - b.x).abs() as f32;
    let dy = (a.y - b.y).abs() as f32;
    let min_d = dx.min(dy);
    let max_d = dx.max(dy);
    (std::f32::consts::SQRT_2 - 1.0) * min_d + max_d
}

fn find_successors(grid: &Grid2D, current: GridPos, parent: Option<GridPos>, goal: GridPos) -> Vec<GridPos> {
    let mut successors = Vec::new();
    let neighbors = prune_neighbors(grid, current, parent);
    
    for n in neighbors {
        let dx = n.x - current.x;
        let dy = n.y - current.y;
        
        if let Some(jump_point) = jump(grid, current, dx, dy, goal) {
            successors.push(jump_point);
        }
    }
    successors
}

fn prune_neighbors(grid: &Grid2D, current: GridPos, parent: Option<GridPos>) -> Vec<GridPos> {
    let mut neighbors = Vec::new();
    
    if parent.is_none() {
        let dirs = [
            (0, 1), (1, 0), (0, -1), (-1, 0), 
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ];
        for (dx, dy) in dirs.iter() {
            if is_walkable(grid, current.x + dx, current.y + dy) {
                 if *dx != 0 && *dy != 0 {
                     if !grid.diagonal_movement_allowed(current, *dx, *dy) { continue; }
                 }
                 neighbors.push(GridPos { x: current.x + dx, y: current.y + dy });
            }
        }
        return neighbors;
    }
    
    let parent = parent.unwrap();
    let dx = (current.x - parent.x).signum();
    let dy = (current.y - parent.y).signum();
    
    if dx != 0 && dy != 0 {
        if is_walkable(grid, current.x + dx, current.y + dy) {
             if grid.diagonal_movement_allowed(current, dx, dy) {
                neighbors.push(GridPos { x: current.x + dx, y: current.y + dy });
             }
        }
        if is_walkable(grid, current.x + dx, current.y) {
             neighbors.push(GridPos { x: current.x + dx, y: current.y });
        }
        if is_walkable(grid, current.x, current.y + dy) {
             neighbors.push(GridPos { x: current.x, y: current.y + dy });
        }
        
        if !is_walkable(grid, current.x - dx, current.y) && is_walkable(grid, current.x - dx, current.y + dy) {
            if grid.diagonal_movement_allowed(GridPos { x: current.x - dx, y: current.y }, 0, dy) { // Loose check? No, move is diagonal from current?
               // The move is from current to (current.x - dx, current.y + dy).
               // Direction is (-dx, dy).
               if grid.diagonal_movement_allowed(current, -dx, dy) {
                   neighbors.push(GridPos { x: current.x - dx, y: current.y + dy });
               }
            }
        }
        if !is_walkable(grid, current.x, current.y - dy) && is_walkable(grid, current.x + dx, current.y - dy) {
             if grid.diagonal_movement_allowed(current, dx, -dy) {
                neighbors.push(GridPos { x: current.x + dx, y: current.y - dy });
             }
        }
    } else if dx != 0 {
        if is_walkable(grid, current.x + dx, current.y) {
             neighbors.push(GridPos { x: current.x + dx, y: current.y });
        }
        if !is_walkable(grid, current.x, current.y + 1) && is_walkable(grid, current.x + dx, current.y + 1) {
             if grid.diagonal_movement_allowed(current, dx, 1) {
                neighbors.push(GridPos { x: current.x + dx, y: current.y + 1 });
             }
        }
        if !is_walkable(grid, current.x, current.y - 1) && is_walkable(grid, current.x + dx, current.y - 1) {
             if grid.diagonal_movement_allowed(current, dx, -1) {
                neighbors.push(GridPos { x: current.x + dx, y: current.y - 1 });
             }
        }
    } else {
         if is_walkable(grid, current.x, current.y + dy) {
             neighbors.push(GridPos { x: current.x, y: current.y + dy });
        }
        if !is_walkable(grid, current.x + 1, current.y) && is_walkable(grid, current.x + 1, current.y + dy) {
             if grid.diagonal_movement_allowed(current, 1, dy) {
                neighbors.push(GridPos { x: current.x + 1, y: current.y + dy });
             }
        }
        if !is_walkable(grid, current.x - 1, current.y) && is_walkable(grid, current.x - 1, current.y + dy) {
             if grid.diagonal_movement_allowed(current, -1, dy) {
                neighbors.push(GridPos { x: current.x - 1, y: current.y + dy });
             }
        }
    }
    
    neighbors
}

fn jump(grid: &Grid2D, current: GridPos, dx: i32, dy: i32, goal: GridPos) -> Option<GridPos> {
    let next_x = current.x + dx;
    let next_y = current.y + dy;
    
    if !is_walkable(grid, next_x, next_y) {
        return None;
    }
    
    if dx != 0 && dy != 0 {
         if !grid.diagonal_movement_allowed(current, dx, dy) { return None; }
    }
    
    let next_node = GridPos { x: next_x, y: next_y };
    
    if next_node == goal {
        return Some(next_node);
    }
    
    if dx != 0 && dy != 0 {
         if (!is_walkable(grid, next_x - dx, next_y) && is_walkable(grid, next_x - dx, next_y + dy)) ||
            (!is_walkable(grid, next_x, next_y - dy) && is_walkable(grid, next_x + dx, next_y - dy)) {
            return Some(next_node);
        }
        if jump(grid, next_node, dx, 0, goal).is_some() || jump(grid, next_node, 0, dy, goal).is_some() {
            return Some(next_node);
        }
    } else if dx != 0 {
        if (!is_walkable(grid, next_x, next_y + 1) && is_walkable(grid, next_x + dx, next_y + 1)) ||
           (!is_walkable(grid, next_x, next_y - 1) && is_walkable(grid, next_x + dx, next_y - 1)) {
            return Some(next_node);
        }
    } else {
        if (!is_walkable(grid, next_x + 1, next_y) && is_walkable(grid, next_x + 1, next_y + dy)) ||
           (!is_walkable(grid, next_x - 1, next_y) && is_walkable(grid, next_x - 1, next_y + dy)) {
            return Some(next_node);
        }
    }
    
    jump(grid, next_node, dx, dy, goal)
}

fn is_walkable(grid: &Grid2D, x: i32, y: i32) -> bool {
    !grid.is_blocked(x, y)
}

trait GridExt {
    fn diagonal_movement_allowed(&self, from: GridPos, dx: i32, dy: i32) -> bool;
}

impl GridExt for Grid2D {
    fn diagonal_movement_allowed(&self, from: GridPos, dx: i32, dy: i32) -> bool {
        if self.diagonal_movement == DiagonalMode::Never { return false; }
        let c1_blocked = self.is_blocked(from.x + dx, from.y);
        let c2_blocked = self.is_blocked(from.x, from.y + dy);
        
         match self.diagonal_movement {
            DiagonalMode::Never => false,
            DiagonalMode::Always => true,
            DiagonalMode::IfNoObstacle => !c1_blocked || !c2_blocked,
            DiagonalMode::OnlyIfBothOpen => !c1_blocked && !c2_blocked,
        }
    }
}

fn reconstruct_path<N: Clone + Eq + std::hash::Hash>(
    current: N,
    came_from: &HashMap<N, N>,
    cost: f32,
    nodes_expanded: usize,
    status: PathStatus
) -> PathResult<N> {
    let mut path = vec![current.clone()];
    let mut cur = current;
    while let Some(parent) = came_from.get(&cur) {
        path.push(parent.clone());
        cur = parent.clone();
    }
    path.reverse();
    PathResult {
        path,
        cost,
        nodes_expanded,
        status,
    }
}
