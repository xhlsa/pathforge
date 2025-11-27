use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;
use std::time::{Duration, Instant};
use std::hash::Hash;
use crate::traits::{Graph, Heuristic, PathResult, PathStatus};

#[derive(Clone, Copy)]
pub struct AStarConfig {
    pub max_iterations: Option<usize>,
    pub timeout: Option<Duration>,
    pub tie_breaking: TieBreaking,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            max_iterations: None,
            timeout: None,
            tie_breaking: TieBreaking::PreferHigherG, // Defaulting to standard best practice
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TieBreaking {
    None,
    PreferHigherG,   // Prefer paths further from start (closer to goal usually)
    PreferLowerG,    // Prefer paths closer to start
    CrossProduct,    // Break ties toward goal direction (Not implemented in heap sort yet)
}

#[derive(Clone, Copy)]
struct State<N> {
    node: N,
    cost: f32, // f_score
    g_score: f32, // Actual cost from start
    tie_breaker: f32, // Secondary sort key derived from strategy
}

impl<N: Eq> PartialEq for State<N> {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.tie_breaker == other.tie_breaker
    }
}

impl<N: Eq> Eq for State<N> {}

// BinaryHeap is max-heap. We want MIN-heap for cost.
// So we invert the comparison for cost.
// For tie_breaker:
// If we want Higher G (PreferHigherG) to come first, and tie_breaker = g_score...
// Higher G means "closer to goal" usually.
// Standard A* tie breaking: if f values are equal, prefer node with higher g-cost (lower h-cost).
// So we want the one with HIGHER tie_breaker to be considered "smaller" (popped sooner)? 
// Wait. BinaryHeap pops the GREATEST element.
// To pop the Best node (Smallest f), we say: if self.cost < other.cost => Greater (so it floats up).
// If self.cost == other.cost:
// We want the one with Higher G to float up.
// So if self.tie_breaker > other.tie_breaker => Greater.
impl<N: Eq> Ord for State<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.cost < other.cost {
            return Ordering::Greater;
        } else if self.cost > other.cost {
            return Ordering::Less;
        }
        
        // Costs are equal, check tie breaker.
        // We want higher tie_breaker to be "Greater" so it is popped first.
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

pub fn astar<G, H>(
    graph: &G,
    heuristic: &H,
    start: G::Node,
    goal: G::Node,
    config: AStarConfig,
) -> PathResult<G::Node>
where
    G: Graph,
    H: Heuristic<G::Node>,
{
    let start_time = Instant::now();
    let mut open_set = BinaryHeap::new();
    let mut g_scores = HashMap::new();
    let mut came_from = HashMap::new();
    
    g_scores.insert(start.clone(), 0.0);
    
    let h_start = heuristic.estimate(&start, &goal);
    open_set.push(State {
        node: start.clone(),
        cost: h_start,
        g_score: 0.0,
        tie_breaker: 0.0, // Start node tie breaking irrelevant usually
    });
    
    let mut nodes_expanded = 0;
    let mut iterations = 0;
    
    while let Some(State { node: current, cost: _f_score, g_score: current_g, tie_breaker: _ }) = open_set.pop() {
        iterations += 1;
        
        // Check limits
        if let Some(max_iter) = config.max_iterations {
            if iterations > max_iter {
                 return reconstruct_partial(current, &came_from, current_g, nodes_expanded, PathStatus::PartialMaxIter);
            }
        }
        if let Some(timeout) = config.timeout {
            if start_time.elapsed() > timeout {
                return reconstruct_partial(current, &came_from, current_g, nodes_expanded, PathStatus::PartialTimeout);
            }
        }
        
        if current == goal {
            return reconstruct_path(current, &came_from, current_g, nodes_expanded, PathStatus::Found);
        }
        
        // Optimization: Check if we found a better path to this node already
        if let Some(&best_g) = g_scores.get(&current) {
            // If the popped node has a worse (higher) g_score than what is known, it is stale.
            // Using strict inequality > because float equality is tricky, but mostly if we have a strictly better path, this one is >.
            if current_g > best_g {
                continue;
            }
        }
        
        nodes_expanded += 1;

        graph.neighbors(&current, |neighbor, edge_cost| {
            let tentative_g = current_g + edge_cost;
            
            if let Some(&existing_g) = g_scores.get(&neighbor) {
                if tentative_g >= existing_g {
                    return;
                }
            }
            
            came_from.insert(neighbor.clone(), current.clone());
            g_scores.insert(neighbor.clone(), tentative_g);
            
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

fn reconstruct_partial<N: Clone + Eq + Hash>(
    current: N,
    came_from: &HashMap<N, N>,
    cost: f32,
    nodes_expanded: usize,
    status: PathStatus
) -> PathResult<N> {
    reconstruct_path(current, came_from, cost, nodes_expanded, status)
}
