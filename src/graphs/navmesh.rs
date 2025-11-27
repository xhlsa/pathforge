use crate::traits::Graph;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Eq for Vec2 {} // Float Eq is dangerous but required for Hash? 
// No, Graph::Node requires Eq + Hash.
// If we use Polygon ID (usize) as Node, we are fine.
// Vec2 is just data.

#[derive(Clone, Debug)]
pub struct ConvexPolygon {
    pub vertices: Vec<usize>, // Indices into mesh.vertices
    pub center: Vec2,
}

pub struct NavMesh {
    pub vertices: Vec<Vec2>,
    pub polygons: Vec<ConvexPolygon>,
    // Adjacency: poly_index -> list of (neighbor_poly_index, distance_between_centers)
    // In a real navmesh, we'd store the shared edge (portal) too.
    pub adjacency: Vec<Vec<(usize, f32)>>,
}

impl NavMesh {
    pub fn new(vertices: Vec<Vec2>, polygons: Vec<ConvexPolygon>) -> Self {
        let mut mesh = Self {
            vertices,
            polygons,
            adjacency: Vec::new(),
        };
        mesh.build_adjacency();
        mesh
    }
    
    fn build_adjacency(&mut self) {
        let n = self.polygons.len();
        self.adjacency = vec![Vec::new(); n];
        
        // Brute force finding shared edges (optimize with edge map in production)
        for i in 0..n {
            for j in (i+1)..n {
                if self.share_edge(i, j) {
                    let dist = self.dist(self.polygons[i].center, self.polygons[j].center);
                    self.adjacency[i].push((j, dist));
                    self.adjacency[j].push((i, dist));
                }
            }
        }
    }
    
    fn share_edge(&self, poly_a: usize, poly_b: usize) -> bool {
        let a = &self.polygons[poly_a];
        let b = &self.polygons[poly_b];
        
        let mut shared_verts = 0;
        for &va in &a.vertices {
            if b.vertices.contains(&va) {
                shared_verts += 1;
            }
        }
        shared_verts >= 2
    }
    
    fn dist(&self, a: Vec2, b: Vec2) -> f32 {
        let dx = a.x - b.x;
        let dy = a.y - b.y;
        (dx*dx + dy*dy).sqrt()
    }
}

impl Graph for NavMesh {
    type Node = usize; // Polygon Index
    
    fn is_passable(&self, node: &Self::Node) -> bool {
        *node < self.polygons.len()
    }
    
    fn neighbors<F>(&self, node: &Self::Node, mut visit: F)
    where
        F: FnMut(Self::Node, f32),
    {
        if let Some(adj) = self.adjacency.get(*node) {
            for &(neighbor, cost) in adj {
                visit(neighbor, cost);
            }
        }
    }
}

// Helper to convert polygon path to point path (centroids)
pub fn polygons_to_centroids(mesh: &NavMesh, path: &[usize]) -> Vec<Vec2> {
    path.iter().map(|&idx| mesh.polygons[idx].center).collect()
}
