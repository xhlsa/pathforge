use crate::graphs::grid2d::{Grid2D, GridPos};
use crate::traits::Graph;
use std::collections::VecDeque;

pub struct FlowField {
    pub width: usize,
    pub height: usize,
    pub integration_field: Vec<f32>,
    pub vector_field: Vec<(f32, f32)>, // Normalized direction vector
}

impl FlowField {
    pub fn generate(grid: &Grid2D, goal: GridPos) -> Self {
        let mut integration = vec![f32::INFINITY; grid.width * grid.height];
        let mut queue = VecDeque::new();
        
        // Validate goal
        if goal.x < 0 || goal.y < 0 || goal.x as usize >= grid.width || goal.y as usize >= grid.height {
             // Return empty flow field?
             return Self {
                 width: grid.width,
                 height: grid.height,
                 integration_field: integration,
                 vector_field: vec![(0.0, 0.0); grid.width * grid.height],
             };
        }
        
        let goal_idx = goal.y as usize * grid.width + goal.x as usize;
        integration[goal_idx] = 0.0;
        queue.push_back(goal);
        
        // Dijkstra / BFS
        while let Some(current) = queue.pop_front() {
            let idx = current.y as usize * grid.width + current.x as usize;
            let current_cost = integration[idx];
            
            grid.neighbors(&current, |neighbor, edge_cost| {
                 let n_idx = neighbor.y as usize * grid.width + neighbor.x as usize;
                 let new_cost = current_cost + edge_cost;
                 if integration[n_idx] > new_cost {
                     integration[n_idx] = new_cost;
                     queue.push_back(neighbor);
                 }
            });
        }
        
        // Compute Vector Field
        let mut vectors = vec![(0.0, 0.0); grid.width * grid.height];
        
        for y in 0..grid.height {
            for x in 0..grid.width {
                let idx = y * grid.width + x;
                if integration[idx] == f32::INFINITY { continue; }
                if x == goal.x as usize && y == goal.y as usize { continue; } // Goal has 0 vector
                
                let mut best_cost = integration[idx];
                let mut best_dir = (0.0, 0.0);
                
                // Check neighbors to find steepest descent
                 grid.neighbors(&GridPos{x: x as i32, y: y as i32}, |n, _cost| {
                     let n_idx = n.y as usize * grid.width + n.x as usize;
                     if integration[n_idx] < best_cost {
                         best_cost = integration[n_idx];
                         best_dir = ((n.x - x as i32) as f32, (n.y - y as i32) as f32);
                     }
                 });
                 
                 // Normalize
                 let len = (best_dir.0 * best_dir.0 + best_dir.1 * best_dir.1).sqrt();
                 if len > 0.0 {
                     vectors[idx] = (best_dir.0 / len, best_dir.1 / len);
                 }
            }
        }
        
        Self {
            width: grid.width,
            height: grid.height,
            integration_field: integration,
            vector_field: vectors,
        }
    }
    
    pub fn get_direction(&self, x: usize, y: usize) -> (f32, f32) {
        if x >= self.width || y >= self.height { return (0.0, 0.0); }
        self.vector_field[y * self.width + x]
    }
}
