use std::collections::{BinaryHeap, HashMap};
use std::time::{Duration, Instant};
use std::hash::Hash;
use std::cmp::Ordering;
use crate::traits::{Graph, Heuristic, PathResult, PathStatus};
use crate::algorithms::astar::{TieBreaking, AStarConfig};

#[derive(Clone, Copy)]
struct State<N> {
    node: N,
    cost: f32,
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
        if self.cost < other.cost { return Ordering::Greater; }
        else if self.cost > other.cost { return Ordering::Less; }
        if self.tie_breaker > other.tie_breaker { return Ordering::Greater; }
        else if self.tie_breaker < other.tie_breaker { return Ordering::Less; }
        Ordering::Equal
    }
}
impl<N: Eq> PartialOrd for State<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

pub enum ComputeStatus<N> {
    NotStarted,
    InProgress,
    Complete(PathResult<N>),
}

pub struct BudgetedPathfinder<G: Graph> {
    open_set: BinaryHeap<State<G::Node>>,
    g_scores: HashMap<G::Node, f32>,
    came_from: HashMap<G::Node, G::Node>,
    goal: Option<G::Node>,
    config: AStarConfig,
    nodes_expanded: usize,
    iterations: usize,
    pub status: ComputeStatus<G::Node>,
    pub last_partial: Option<PathResult<G::Node>>,
}

impl<G> BudgetedPathfinder<G> 
where 
    G: Graph,
    G::Node: Clone + Eq + Hash + std::fmt::Debug,
{
    pub fn new(config: AStarConfig) -> Self {
        Self {
            open_set: BinaryHeap::new(),
            g_scores: HashMap::new(),
            came_from: HashMap::new(),
            goal: None,
            config,
            nodes_expanded: 0,
            iterations: 0,
            status: ComputeStatus::NotStarted,
            last_partial: None,
        }
    }

    pub fn start<H>(&mut self, start: G::Node, goal: G::Node, heuristic: &H)
    where H: Heuristic<G::Node> {
        self.open_set.clear();
        self.g_scores.clear();
        self.came_from.clear();
        self.nodes_expanded = 0;
        self.iterations = 0;
        self.last_partial = None;
        
        self.g_scores.insert(start.clone(), 0.0);
        let h = heuristic.estimate(&start, &goal);
        
        self.open_set.push(State {
            node: start.clone(),
            cost: h,
            g_score: 0.0,
            tie_breaker: 0.0,
        });
        
        self.goal = Some(goal);
        self.status = ComputeStatus::InProgress;
    }
    
    pub fn step<H>(&mut self, graph: &G, heuristic: &H, budget: Duration) -> bool 
    where H: Heuristic<G::Node> {
        if let ComputeStatus::Complete(_) = self.status {
            return true;
        }
        if self.goal.is_none() { return true; }
        
        let start_time = Instant::now();
        let goal = self.goal.as_ref().unwrap();
        
        while let Some(State { node: current, cost: f_score, g_score: current_g, tie_breaker: tb }) = self.open_set.pop() {
            self.iterations += 1;
             
             // Check budget
            if self.iterations % 10 == 0 && start_time.elapsed() > budget {
                 self.last_partial = Some(self.reconstruct_path(current.clone(), PathStatus::PartialTimeout));
                 self.open_set.push(State { node: current, cost: f_score, g_score: current_g, tie_breaker: tb }); 
                 return false; 
             }

            if &current == goal {
                let res = self.reconstruct_path(current, PathStatus::Found);
                self.last_partial = Some(res.clone());
                self.status = ComputeStatus::Complete(res);
                return true;
            }
            
            if let Some(&best_g) = self.g_scores.get(&current) {
                if current_g > best_g { continue; }
            }
            
            self.nodes_expanded += 1;
            
             graph.neighbors(&current, |neighbor, edge_cost| {
                let tentative_g = current_g + edge_cost;
                if let Some(&existing_g) = self.g_scores.get(&neighbor) {
                    if tentative_g >= existing_g { return; }
                }
                
                self.came_from.insert(neighbor.clone(), current.clone());
                self.g_scores.insert(neighbor.clone(), tentative_g);
                
                let h = heuristic.estimate(&neighbor, goal);
                let f = tentative_g + h;
                let tb = match self.config.tie_breaking {
                    TieBreaking::None => 0.0,
                    TieBreaking::PreferHigherG => tentative_g,
                    TieBreaking::PreferLowerG => -tentative_g,
                    TieBreaking::CrossProduct => 0.0,
                };
                
                self.open_set.push(State { node: neighbor, cost: f, g_score: tentative_g, tie_breaker: tb });
            });
        }
        
        // Open set empty, path not found
        self.status = ComputeStatus::Complete(PathResult {
            path: vec![],
            cost: 0.0,
            nodes_expanded: self.nodes_expanded,
            status: PathStatus::NotFound,
        });
        self.last_partial = None;
        true
    }
    
    fn reconstruct_path(&self, current: G::Node, status: PathStatus) -> PathResult<G::Node> {
        let mut path = vec![current.clone()];
        let mut cur = current;
        while let Some(parent) = self.came_from.get(&cur) {
            path.push(parent.clone());
            cur = parent.clone();
        }
        path.reverse();
        
        let cost = path.last()
            .and_then(|node| self.g_scores.get(node))
            .copied()
            .unwrap_or(0.0);

        PathResult {
            path,
            cost,
            nodes_expanded: self.nodes_expanded,
            status,
        }
    }
    
    pub fn take_result(&mut self) -> Option<PathResult<G::Node>> {
        if let ComputeStatus::Complete(ref res) = self.status {
            Some(res.clone())
        } else {
            None
        }
    }

    pub fn partial_result(&self) -> Option<&PathResult<G::Node>> {
        self.last_partial.as_ref()
    }
}
