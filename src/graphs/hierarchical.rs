use crate::graphs::grid2d::{Grid2D, GridPos};
use crate::algorithms::astar::{astar, AStarConfig};
use crate::heuristics::{Euclidean, Manhattan};
use crate::traits::{Graph, Heuristic, PathResult, PathStatus};
use std::collections::HashMap;
use rayon::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct AbstractNodeId(usize);

#[derive(Debug, Clone)]
pub struct AbstractEdge {
    pub target: AbstractNodeId,
    pub cost: f32,
    // We cache the actual path steps to avoid recomputing them during refinement
    pub path: Vec<GridPos>, 
}

pub struct HierarchicalGrid {
    pub base_grid: Grid2D,
    pub cluster_size: usize,
    
    // The Abstract Graph
    pub nodes: Vec<GridPos>, // ID -> Real Position
    pub edges: HashMap<AbstractNodeId, Vec<AbstractEdge>>,
    
    // Lookups
    // (cluster_x, cluster_y) -> List of Abstract Nodes belonging to this cluster
    cluster_nodes: HashMap<(usize, usize), Vec<AbstractNodeId>>,
}

impl HierarchicalGrid {
    pub fn new(base_grid: Grid2D, cluster_size: usize) -> Self {
        let mut hp = Self {
            base_grid,
            cluster_size,
            nodes: Vec::new(),
            edges: HashMap::new(),
            cluster_nodes: HashMap::new(),
        };
        hp.preprocess();
        hp
    }

    fn preprocess(&mut self) {
        self.build_abstract_nodes();
        self.build_intra_cluster_edges();
    }

    fn build_abstract_nodes(&mut self) {
        let w = self.base_grid.width;
        let h = self.base_grid.height;
        let cs = self.cluster_size;
        
        let cluster_cols = (w + cs - 1) / cs;
        let cluster_rows = (h + cs - 1) / cs;

        // Vertical borders (between cluster (cx, cy) and (cx+1, cy))
        for cy in 0..cluster_rows {
            for cx in 0..cluster_cols - 1 {
                let px = (cx + 1) * cs - 1; // Left side of border
                let px_next = px + 1;       // Right side of border
                
                if px_next >= w { continue; }

                let y_start = cy * cs;
                let y_end = ((cy + 1) * cs).min(h);

                self.detect_entrances(
                    px, 
                    y_start, 
                    y_end, 
                    true, 
                    px_next
                );
            }
        }

        // Horizontal borders (between cluster (cx, cy) and (cx, cy+1))
        for cy in 0..cluster_rows - 1 {
            for cx in 0..cluster_cols {
                let py = (cy + 1) * cs - 1;
                let py_next = py + 1;
                
                if py_next >= h { continue; }

                let x_start = cx * cs;
                let x_end = ((cx + 1) * cs).min(w);

                self.detect_entrances(
                    py, 
                    x_start, 
                    x_end, 
                    false, 
                    py_next
                );
            }
        }
    }

    // Scans a border line for contiguous passable segments
    fn detect_entrances(&mut self, fixed_coord: usize, range_start: usize, range_end: usize, is_vertical: bool, neighbor_coord: usize) {
        let mut start_idx = None;

        for i in range_start..range_end {
            let (c1_x, c1_y, c2_x, c2_y) = if is_vertical {
                (fixed_coord, i, neighbor_coord, i)
            } else {
                (i, fixed_coord, i, neighbor_coord)
            };

            let passable = !self.base_grid.is_blocked(c1_x as i32, c1_y as i32) 
                        && !self.base_grid.is_blocked(c2_x as i32, c2_y as i32);

            if passable {
                if start_idx.is_none() {
                    start_idx = Some(i);
                }
            } else {
                if let Some(s) = start_idx {
                    self.create_entrance(s, i - 1, fixed_coord, is_vertical, neighbor_coord);
                    start_idx = None;
                }
            }
        }

        if let Some(s) = start_idx {
            self.create_entrance(s, range_end - 1, fixed_coord, is_vertical, neighbor_coord);
        }
    }

    fn create_entrance(&mut self, start: usize, end: usize, fixed: usize, is_vertical: bool, neighbor_fixed: usize) {
        // Place one node at the center of the entrance
        let mid = (start + end) / 2;
        
        let (pos1, pos2) = if is_vertical {
            (GridPos { x: fixed as i32, y: mid as i32 }, GridPos { x: neighbor_fixed as i32, y: mid as i32 })
        } else {
            (GridPos { x: mid as i32, y: fixed as i32 }, GridPos { x: mid as i32, y: neighbor_fixed as i32 })
        };

        let id1 = self.add_node(pos1);
        let id2 = self.add_node(pos2);

        // Add "Inter-edge" (cost 1.0, immediate neighbor)
        self.add_edge(id1, id2, 1.0, vec![pos1, pos2]);
        self.add_edge(id2, id1, 1.0, vec![pos2, pos1]);
    }

    fn add_node(&mut self, pos: GridPos) -> AbstractNodeId {
        let id = AbstractNodeId(self.nodes.len());
        self.nodes.push(pos);
        self.edges.insert(id, Vec::new());
        
        let cx = pos.x as usize / self.cluster_size;
        let cy = pos.y as usize / self.cluster_size;
        self.cluster_nodes.entry((cx, cy)).or_default().push(id);
        
        id
    }

    fn add_edge(&mut self, from: AbstractNodeId, to: AbstractNodeId, cost: f32, path: Vec<GridPos>) {
        self.edges.get_mut(&from).unwrap().push(AbstractEdge { target: to, cost, path });
    }

    fn process_cluster(&self, cluster_coords: &(usize, usize)) -> Vec<(AbstractNodeId, AbstractNodeId, f32, Vec<GridPos>)> {
        let mut local_edges = Vec::new();
        let heuristic = Manhattan;

        if let Some(nodes) = self.cluster_nodes.get(cluster_coords) {
            if nodes.len() >= 2 {
                for i in 0..nodes.len() {
                    for j in (i + 1)..nodes.len() {
                        let id_a = nodes[i];
                        let id_b = nodes[j];
                        let pos_a = self.nodes[id_a.0];
                        let pos_b = self.nodes[id_b.0];

                        let result = astar(
                            &self.base_grid, 
                            &heuristic, 
                            pos_a, 
                            pos_b, 
                            AStarConfig::default()
                        );

                        if result.status == PathStatus::Found {
                            local_edges.push((id_a, id_b, result.cost, result.path.clone()));
                            let mut rev_path = result.path;
                            rev_path.reverse();
                            local_edges.push((id_b, id_a, result.cost, rev_path));
                        }
                    }
                }
            }
        }
        local_edges
    }

    fn build_intra_cluster_edges(&mut self) {
        let clusters: Vec<(usize, usize)> = self.cluster_nodes.keys().cloned().collect();
        
        // Use parallel execution only if we have enough work (threshold > 50 clusters)
        let new_edges: Vec<(AbstractNodeId, AbstractNodeId, f32, Vec<GridPos>)> = if clusters.len() > 50 {
            clusters.par_iter()
                .flat_map(|c| self.process_cluster(c))
                .collect()
        } else {
            clusters.iter()
                .flat_map(|c| self.process_cluster(c))
                .collect()
        };

        for (from, to, cost, path) in new_edges {
            self.add_edge(from, to, cost, path);
        }
    }
    
    pub fn find_path(&self, start: GridPos, goal: GridPos) -> PathResult<GridPos> {
        // 1. Insert Start and Goal as temporary nodes
        // But we can't modify self. So we build a temporary graph wrapper or
        // just do the logic ad-hoc. Ad-hoc is easier for this snippet.
        
        // Start cluster
        let s_cx = start.x as usize / self.cluster_size;
        let s_cy = start.y as usize / self.cluster_size;
        
        // Goal cluster
        let g_cx = goal.x as usize / self.cluster_size;
        let g_cy = goal.y as usize / self.cluster_size;
        
        // If same cluster, just run normal A*
        if s_cx == g_cx && s_cy == g_cy {
             return astar(&self.base_grid, &Euclidean, start, goal, AStarConfig::default());
        }

        // 2. Connect Start to its cluster's abstract nodes
        let mut start_edges: Vec<(AbstractNodeId, f32, Vec<GridPos>)> = Vec::new();
        if let Some(nodes) = self.cluster_nodes.get(&(s_cx, s_cy)) {
            for &target_id in nodes {
                let target_pos = self.nodes[target_id.0];
                let res = astar(&self.base_grid, &Euclidean, start, target_pos, AStarConfig::default());
                if res.status == PathStatus::Found {
                    start_edges.push((target_id, res.cost, res.path));
                }
            }
        }

        // 3. Connect Goal to its cluster's abstract nodes (incoming)
        // Effectively we want edges FROM abstract nodes TO goal.
        // Since graph is undirected (mostly), we calculate From Goal To Abstract and reverse.
        let mut goal_edges: Vec<(AbstractNodeId, f32, Vec<GridPos>)> = Vec::new();
        if let Some(nodes) = self.cluster_nodes.get(&(g_cx, g_cy)) {
            for &src_id in nodes {
                let src_pos = self.nodes[src_id.0];
                let res = astar(&self.base_grid, &Euclidean, src_pos, goal, AStarConfig::default());
                if res.status == PathStatus::Found {
                    goal_edges.push((src_id, res.cost, res.path));
                }
            }
        }

        // 4. Run A* on Abstract Graph
        // Nodes: 0..N (abstract), N+1 (Start), N+2 (Goal)
        // We use a custom Graph struct for the search
        
        let start_id_virtual = AbstractNodeId(usize::MAX - 1);
        let goal_id_virtual = AbstractNodeId(usize::MAX);
        
        struct AbstractSearchGraph<'a> {
            hp: &'a HierarchicalGrid,
            start_edges: &'a [(AbstractNodeId, f32, Vec<GridPos>)],
            goal_edges: &'a [(AbstractNodeId, f32, Vec<GridPos>)],
            #[allow(dead_code)]
            start_pos: GridPos,
            #[allow(dead_code)]
            goal_pos: GridPos,
            start_id: AbstractNodeId,
            goal_id: AbstractNodeId,
        }
        
        impl<'a> Graph for AbstractSearchGraph<'a> {
            type Node = AbstractNodeId;
            
            fn is_passable(&self, _node: &Self::Node) -> bool { true }
            
            fn neighbors<F>(&self, node: &Self::Node, mut visit: F)
            where F: FnMut(Self::Node, f32) {
                if *node == self.start_id {
                    for (target, cost, _) in self.start_edges {
                        visit(*target, *cost);
                    }
                } else if *node == self.goal_id {
                    // Goal has no outgoing neighbors
                } else {
                    // Real abstract node
                    if let Some(edges) = self.hp.edges.get(node) {
                        for edge in edges {
                            visit(edge.target, edge.cost);
                        }
                    }
                    // Also check if we can reach goal directly
                    // (This is the reverse of goal_edges)
                    for (src, cost, _) in self.goal_edges {
                        if src == node {
                            visit(self.goal_id, *cost);
                        }
                    }
                }
            }
        }
        
        struct AbstractHeuristic<'a> {
            hp: &'a HierarchicalGrid,
            goal_pos: GridPos,
            goal_id: AbstractNodeId,
        }
        
        impl<'a> Heuristic<AbstractNodeId> for AbstractHeuristic<'a> {
            fn estimate(&self, from: &AbstractNodeId, _to: &AbstractNodeId) -> f32 {
                // We always estimate to the real goal position
                let from_pos = if *from == AbstractNodeId(usize::MAX - 1) {
                    // Start virtual node - strictly speaking heuristic not needed for start,
                    // but let's say dist is large. 
                    // Actually A* calls h(start, goal).
                    // We don't store start pos in Heuristic? We can.
                    // But we only need from_pos.
                    return 0.0; // Hacky?
                } else if *from == self.goal_id {
                    return 0.0;
                } else {
                    self.hp.nodes[from.0]
                };
                
                // Euclidean
                let dx = (from_pos.x - self.goal_pos.x) as f32;
                let dy = (from_pos.y - self.goal_pos.y) as f32;
                (dx*dx + dy*dy).sqrt()
            }
        }

        let search_graph = AbstractSearchGraph {
            hp: self,
            start_edges: &start_edges,
            goal_edges: &goal_edges,
            start_pos: start,
            goal_pos: goal,
            start_id: start_id_virtual,
            goal_id: goal_id_virtual,
        };

        // Heuristic needs to handle the virtual start node correctly if called
        // But usually we map IDs.
        // Let's just use 0.0 for virtual nodes in a wrapper.
        // Actually, we can just implement Heuristic for AbstractSearchGraph
        let search_heuristic = AbstractHeuristic {
            hp: self,
            goal_pos: goal,
            goal_id: goal_id_virtual,
        };
        
        let abstract_result = astar(
            &search_graph,
            &search_heuristic,
            start_id_virtual,
            goal_id_virtual,
            AStarConfig::default()
        );
        
        if abstract_result.status != PathStatus::Found {
             return PathResult {
                 path: vec![],
                 cost: 0.0,
                 nodes_expanded: abstract_result.nodes_expanded,
                 status: abstract_result.status,
             };
        }
        
        // 5. Reconstruct High-Level path to Low-Level
        let mut full_path = Vec::new();
        
        // Abstract path: [StartVirtual, NodeA, NodeB, ..., GoalVirtual]
        let ap = abstract_result.path;
        
        for i in 0..ap.len() - 1 {
            let current = ap[i];
            let next = ap[i+1];
            
            let segment_path: &[GridPos] = if current == start_id_virtual {
                // Start -> Next
                let (_, _, ref p) = start_edges.iter().find(|(id, _, _)| *id == next).unwrap();
                p
            } else if next == goal_id_virtual {
                // Current -> Goal
                let (_, _, ref p) = goal_edges.iter().find(|(id, _, _)| *id == current).unwrap();
                p
            } else {
                // Node -> Node
                let edges = &self.edges[&current];
                let edge = edges.iter().find(|e| e.target == next).unwrap();
                &edge.path
            };
            
            // Append segment (skip first element if not at very beginning to avoid duplication)
            if full_path.is_empty() {
                full_path.extend_from_slice(segment_path);
            } else {
                full_path.extend_from_slice(&segment_path[1..]);
            }
        }
        
        PathResult {
            path: full_path,
            cost: abstract_result.cost,
            nodes_expanded: abstract_result.nodes_expanded, // Note: this doesn't count low-level expansions
            status: PathStatus::Found,
        }
    }
}
