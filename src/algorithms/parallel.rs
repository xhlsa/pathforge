use rayon::prelude::*;
use crate::traits::{Graph, Heuristic, PathResult};
use crate::algorithms::astar::{astar, AStarConfig};

pub fn find_paths_parallel<G, H>(
    graph: &G,
    heuristic: &H,
    queries: &[(G::Node, G::Node)],
    config: AStarConfig,
) -> Vec<PathResult<G::Node>>
where
    G: Graph + Sync,
    G::Node: Send + Sync,
    H: Heuristic<G::Node> + Sync,
{
    queries.par_iter()
        .map(|(start, goal)| {
            astar(graph, heuristic, start.clone(), goal.clone(), config)
        })
        .collect()
}
