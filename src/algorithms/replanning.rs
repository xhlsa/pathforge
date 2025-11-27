use crate::traits::{Graph, PathStatus, Heuristic};
use crate::algorithms::astar::{astar, AStarConfig};

pub struct PathReplanner<N> {
    current_path: Vec<N>,
    current_goal: Option<N>,
    last_replanned_at: Option<std::time::Instant>,
    replan_interval: std::time::Duration,
}

impl<N: Clone + Eq + PartialEq> PathReplanner<N> {
    pub fn new(replan_interval: std::time::Duration) -> Self {
        Self {
            current_path: Vec::new(),
            current_goal: None,
            last_replanned_at: None,
            replan_interval,
        }
    }
    
    /// Returns a new path if replanning occurred, or None if the old path is still valid/kept.
    pub fn update<G, H>(
        &mut self,
        graph: &G,
        heuristic: &H,
        current_pos: N,
        goal_pos: N,
        config: AStarConfig,
    ) -> Option<Vec<N>>
    where
        G: Graph<Node = N>,
        H: Heuristic<N>,
        N: std::hash::Hash + std::fmt::Debug,
    {
        let now = std::time::Instant::now();
        
        let needs_replan = if let Some(ref old_goal) = self.current_goal {
             *old_goal != goal_pos || self.current_path.is_empty()
        } else {
            true
        };
        
        let time_due = if let Some(last) = self.last_replanned_at {
            now.duration_since(last) >= self.replan_interval
        } else {
            true
        };

        // Simple policy: Replan if goal moved OR (time is due AND path is invalid/blocked)
        // Checking path validity is expensive (checking every node).
        // Instead, we might just check the next few steps.
        // For now, we replan if goal changed.
        
        if needs_replan || (time_due && self.should_check_validity(graph, &current_pos)) {
            let result = astar(graph, heuristic, current_pos, goal_pos.clone(), config);
            
            if result.status == PathStatus::Found {
                self.current_path = result.path.clone();
                self.current_goal = Some(goal_pos);
                self.last_replanned_at = Some(now);
                return Some(result.path);
            }
        }
        
        None
    }
    
    fn should_check_validity<G: Graph<Node = N>>(&self, graph: &G, current_pos: &N) -> bool {
        // Check if the immediate next step in current path is blocked
        // We need to find where we are in the path. 
        // This is O(N) search.
        if let Some(idx) = self.current_path.iter().position(|n| *n == *current_pos) {
            if idx + 1 < self.current_path.len() {
                return !graph.is_passable(&self.current_path[idx+1]);
            }
        }
        false
    }
    
    pub fn get_path(&self) -> &[N] {
        &self.current_path
    }
}
