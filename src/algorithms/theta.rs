use crate::traits::{Graph, Heuristic, PathResult, PathStatus};
use crate::algorithms::astar::{AStarConfig, TieBreaking};
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;
use std::hash::Hash;
use std::time::Instant;

#[derive(Clone, Copy)]
struct State<N> {
    node: N,
    cost: f32, // f_score
    g_score: f32,
    tie_breaker: f32,
}

impl<N: Eq> PartialEq for State<N> {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.tie_breaker == other.tie_breaker
    }
}

impl<N: Eq> Eq for State<N> {}

impl<N: Eq> Ord for State<N> {
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

impl<N: Eq> PartialOrd for State<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn theta_star<G, H>(
    graph: &G,
    heuristic: &H,
    start: G::Node,
    goal: G::Node,
    config: AStarConfig,
) -> PathResult<G::Node>
where
    G: Graph,
    H: Heuristic<G::Node>,
    G::Node: Clone + Eq + Hash + std::fmt::Debug,
{
    let start_time = Instant::now();
    let mut open_set = BinaryHeap::new();
    let mut g_scores = HashMap::new();
    let mut came_from = HashMap::new();
    
    g_scores.insert(start.clone(), 0.0);
    came_from.insert(start.clone(), start.clone()); // Parent of start is start
    
    let h_start = heuristic.estimate(&start, &goal);
    open_set.push(State {
        node: start.clone(),
        cost: h_start,
        g_score: 0.0,
        tie_breaker: 0.0,
    });
    
    let mut nodes_expanded = 0;
    let mut iterations = 0;
    
    while let Some(State { node: current, cost: _, g_score: current_g, tie_breaker: _ }) = open_set.pop() {
        iterations += 1;
        
        // Check limits
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

        // To access parent(current), we look it up.
        // For start node, parent is start.
        let parent_of_current = came_from.get(&current).unwrap_or(&current).clone();

        graph.neighbors(&current, |neighbor, edge_cost| {
            // Theta* logic:
            // Check line of sight from parent(current) to neighbor
            // If LOS exists, we can go Parent -> Neighbor directly
            
            let (new_parent, new_g) = if parent_of_current != current && graph.can_traverse(&parent_of_current, &neighbor) {
                // Path 2: Parent -> Neighbor
                // We need exact distance, usually Euclidean. 
                // 'heuristic.estimate' gives distance between any two nodes (usually).
                // Or we assume graph edges are weights. 
                // Since G::neighbors gives us edge_cost (current->neighbor), we don't have parent->neighbor cost directly.
                // But usually Heuristic is essentially a distance function if admissible.
                // Or we can rely on Euclidean heuristic.
                // Let's use heuristic.estimate(parent, neighbor) assuming it returns actual distance (like Euclidean).
                // If Heuristic is Manhattan, this might be wrong for cost but "ok" for logic?
                // Ideally we need a `graph.distance(a, b)` or `heuristic` must be Euclidean for cost calc.
                let dist = heuristic.estimate(&parent_of_current, &neighbor);
                let pg = g_scores.get(&parent_of_current).copied().unwrap_or(0.0);
                (parent_of_current.clone(), pg + dist)
            } else {
                // Path 1: Current -> Neighbor (Standard A*)
                (current.clone(), current_g + edge_cost)
            };
            
            if let Some(&existing_g) = g_scores.get(&neighbor) {
                if new_g >= existing_g {
                    return;
                }
            }
            
            came_from.insert(neighbor.clone(), new_parent);
            g_scores.insert(neighbor.clone(), new_g);
            
            let h = heuristic.estimate(&neighbor, &goal);
            let f = new_g + h;
            
            let tb = match config.tie_breaking {
                TieBreaking::None => 0.0,
                TieBreaking::PreferHigherG => new_g,
                TieBreaking::PreferLowerG => -new_g,
                TieBreaking::CrossProduct => 0.0,
            };
            
            open_set.push(State {
                node: neighbor,
                cost: f,
                g_score: new_g,
                tie_breaker: tb,
            });
        });
    }
    
    PathResult {
        path: vec![],
        cost: 0.0,
        nodes_expanded,
        status: PathStatus::NotFound,
    }
}

fn reconstruct_path<N: Clone + Eq + Hash>(
    current: N,
    came_from: &HashMap<N, N>,
    cost: f32,
    nodes_expanded: usize,
    status: PathStatus
) -> PathResult<N> {
    let mut path = vec![current.clone()];
    let mut cur = current;
    while let Some(parent) = came_from.get(&cur) {
        if *parent == cur { break; } // Start node points to itself
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
