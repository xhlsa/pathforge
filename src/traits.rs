use std::hash::Hash;

/// Represents a searchable graph structure.
/// Implementors define topology; algorithms handle search strategy.
pub trait Graph {
    type Node: Eq + Hash + Clone;
    
    /// Check if a node is passable (fast rejection before expansion)
    fn is_passable(&self, node: &Self::Node) -> bool;
    
    /// Iterate neighbors with their traversal costs.
    /// Uses callback pattern to avoid allocation.
    fn neighbors<F>(&self, node: &Self::Node, visit: F)
    where
        F: FnMut(Self::Node, f32);  // (neighbor, edge_cost)
    
    /// Optional: Check if direct traversal is valid (for line-of-sight checks)
    fn can_traverse(&self, _from: &Self::Node, _to: &Self::Node) -> bool {
        true
    }
}

/// Heuristic function for informed search algorithms.
/// Separate from Graph because heuristic choice is algorithm policy, not topology.
pub trait Heuristic<N> {
    fn estimate(&self, from: &N, to: &N) -> f32;
    
    /// Must return true if heuristic is admissible (never overestimates)
    fn is_admissible(&self) -> bool { true }
}

/// Result of a pathfinding query
#[derive(Debug, Clone)]
pub struct PathResult<N> {
    pub path: Vec<N>,
    pub cost: f32,
    pub nodes_expanded: usize,
    pub status: PathStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathStatus {
    Found,
    NotFound,
    PartialTimeout,  // Hit frame budget, returning best partial
    PartialMaxIter,  // Hit iteration limit
}
