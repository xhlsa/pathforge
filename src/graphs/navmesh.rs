use crate::traits::Graph;
use crate::algorithms::funnel::Portal;

/// A navigation mesh based on a "Struct of Arrays" layout for cache locality.
/// Currently assumes all polygons are triangles.
pub struct NavMesh {
    /// All vertices in the mesh (x, y, z).
    /// Flattened: [x0, y0, z0, x1, y1, z1, ...]
    pub vertices: Vec<f32>,

    /// Indices of vertices that make up each polygon.
    /// Stride 3 for triangles: [t0_v1, t0_v2, t0_v3, t1_v1, ...]
    pub polygons: Vec<u32>,

    /// Adjacency information.
    /// Indices of neighbor polygons for each edge.
    /// Stride 3 for triangles: [t0_n1, t0_n2, t0_n3, ...]
    /// -1 indicates a boundary edge (no neighbor).
    pub neighbors: Vec<i32>,
}

impl NavMesh {
    pub fn new(vertices: Vec<f32>, polygons: Vec<u32>, neighbors: Vec<i32>) -> Self {
        Self {
            vertices,
            polygons,
            neighbors,
        }
    }

    /// Returns the (x, y, z) of a vertex by its index.
    #[inline]
    pub fn get_vertex(&self, index: u32) -> (f32, f32, f32) {
        let i = (index as usize) * 3;
        (self.vertices[i], self.vertices[i+1], self.vertices[i+2])
    }

    fn get_vertex_arr(&self, index: u32) -> [f32; 3] {
        let (x, y, z) = self.get_vertex(index);
        [x, y, z]
    }

    /// Calculates the centroid of a polygon.
    pub fn centroid(&self, poly_index: u32) -> (f32, f32, f32) {
        let i = (poly_index as usize) * 3;
        let v1 = self.get_vertex(self.polygons[i]);
        let v2 = self.get_vertex(self.polygons[i+1]);
        let v3 = self.get_vertex(self.polygons[i+2]);

        (
            (v1.0 + v2.0 + v3.0) / 3.0,
            (v1.1 + v2.1 + v3.1) / 3.0,
            (v1.2 + v2.2 + v3.2) / 3.0,
        )
    }

    fn dist_sq(a: (f32, f32, f32), b: (f32, f32, f32)) -> f32 {
        let dx = a.0 - b.0;
        let dy = a.1 - b.1;
        let dz = a.2 - b.2;
        dx*dx + dy*dy + dz*dz
    }

    /// Converts a path of polygon indices into a list of portals for the funnel algorithm.
    pub fn get_portals(&self, path: &[u32], start_pos: [f32; 3], end_pos: [f32; 3]) -> Vec<Portal> {
        let mut portals = Vec::with_capacity(path.len() + 1);

        // Start Portal (degenerate)
        portals.push(Portal { left: start_pos, right: start_pos });

        for i in 0..path.len() - 1 {
            let curr = path[i];
            let next = path[i+1];

            if let Some((left, right)) = self.find_shared_edge(curr, next) {
                portals.push(Portal { left, right });
            }
        }

        // End Portal (degenerate)
        portals.push(Portal { left: end_pos, right: end_pos });

        portals
    }

    // Helper to find shared edge between two polygons
    // Returns (left_vertex, right_vertex)
    fn find_shared_edge(&self, p1: u32, p2: u32) -> Option<([f32; 3], [f32; 3])> {
         let start_idx = (p1 as usize) * 3;
         
         // Check all 3 neighbors of p1 to find p2
         for i in 0..3 {
             if self.neighbors[start_idx + i] == p2 as i32 {
                 // Found it! The edge is between vertex i and (i+1)%3
                 let v1_idx = self.polygons[start_idx + i];
                 let v2_idx = self.polygons[start_idx + (i + 1) % 3];
                 
                 let v1 = self.get_vertex_arr(v1_idx);
                 let v2 = self.get_vertex_arr(v2_idx);

                 // Winding: v1 -> v2 is CCW edge of p1.
                 // When crossing v1->v2 to leave p1:
                 // v1 is Right, v2 is Left.
                 return Some((v2, v1));
             }
         }
         None
    }

    /// Finds the polygon ID that contains the given position (XZ plane).
    /// Currently uses an O(N) brute-force search. 
    /// TODO: Optimize with a spatial partition (BVH or Grid) for large meshes.
    pub fn get_poly_at_pos(&self, pos: [f32; 3]) -> Option<u32> {
        let num_polys = self.polygons.len() / 3;
        for i in 0..num_polys {
            let idx = (i as u32) * 3;
            let v1 = self.get_vertex_arr(self.polygons[idx as usize]);
            let v2 = self.get_vertex_arr(self.polygons[idx as usize + 1]);
            let v3 = self.get_vertex_arr(self.polygons[idx as usize + 2]);

            if Self::is_point_in_triangle(pos, v1, v2, v3) {
                return Some(i as u32);
            }
        }
        None
    }

    fn is_point_in_triangle(p: [f32; 3], a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> bool {
        fn sign(p1: [f32; 3], p2: [f32; 3], p3: [f32; 3]) -> f32 {
            (p1[0] - p3[0]) * (p2[2] - p3[2]) - (p2[0] - p3[0]) * (p1[2] - p3[2])
        }

        let d1 = sign(p, a, b);
        let d2 = sign(p, b, c);
        let d3 = sign(p, c, a);

        let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
        let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

        !(has_neg && has_pos)
    }
}

impl Graph for NavMesh {
    type Node = u32; // Polygon Index

    fn is_passable(&self, node: &Self::Node) -> bool {
        (*node as usize) * 3 < self.polygons.len()
    }

    fn neighbors<F>(&self, node: &Self::Node, mut visit: F)
    where
        F: FnMut(Self::Node, f32),
    {
        let poly_index = *node as usize;
        let start_index = poly_index * 3;
        
        // Safety check
        if start_index >= self.neighbors.len() {
            return;
        }

        let center_current = self.centroid(*node);

        // Check all 3 edges
        for i in 0..3 {
            let neighbor_idx = self.neighbors[start_index + i];
            if neighbor_idx != -1 {
                let neighbor_u32 = neighbor_idx as u32;
                let center_next = self.centroid(neighbor_u32);
                
                // Cost: Distance between centroids (Approximation for A*)
                // TODO: Use edge midpoints for more accurate traversal cost
                let cost = Self::dist_sq(center_current, center_next).sqrt();
                
                visit(neighbor_u32, cost);
            }
        }
    }
}

