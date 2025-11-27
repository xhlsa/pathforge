use pathforge::algorithms::astar::{astar, AStarConfig};
use pathforge::algorithms::funnel::string_pull;
use pathforge::graphs::navmesh::NavMesh;
use pathforge::traits::Heuristic;

struct NavMeshHeuristic<'a> {
    mesh: &'a NavMesh,
}

impl<'a> Heuristic<u32> for NavMeshHeuristic<'a> {
    fn estimate(&self, from: &u32, to: &u32) -> f32 {
        let c1 = self.mesh.centroid(*from);
        let c2 = self.mesh.centroid(*to);
        let dx = c1.0 - c2.0;
        let dy = c1.1 - c2.1;
        let dz = c1.2 - c2.2;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

fn main() {
    // Create a simple NavMesh: A corridor of 3 triangles
    // T0: (0,0,0), (2,0,0), (1,0,2) - Base at z=0
    // T1: (1,0,2), (2,0,0), (3,0,2) - Connected to T0 via (2,0,0)-(1,0,2) ??? Wait, let's make it simpler
    
    // Simple square corridor made of 2 triangles forming a quad (0,0) to (2,2)
    // V0:(0,0), V1:(2,0), V2:(2,2), V3:(0,2)
    // T0: 0-1-2
    // T1: 0-2-3
    
    // Vertices (x, y, z) - y is up, so z is depth
    let vertices = vec![
        0.0, 0.0, 0.0, // v0
        2.0, 0.0, 0.0, // v1
        2.0, 0.0, 2.0, // v2
        0.0, 0.0, 2.0, // v3
    ];
    
    // Polygons (indices)
    let polygons = vec![
        0, 1, 2, // T0
        0, 2, 3, // T1
    ];
    
    // Neighbors
    // T0 neighbors: [-1, T1, -1] (Edge 1-2 is shared with T1? No, 1-2 is diagonal)
    // Edge 0: v0-v1 (Bottom) -> None
    // Edge 1: v1-v2 (Right) -> None
    // Edge 2: v2-v0 (Diagonal) -> T1
    
    // T1 neighbors:
    // Edge 0: v0-v2 (Diagonal, matches T0's Edge 2) -> T0
    // Edge 1: v2-v3 (Top) -> None
    // Edge 2: v3-v0 (Left) -> None
    
    let neighbors = vec![
        -1, -1, 1, // T0
        0, -1, -1, // T1
    ];

    let mesh = NavMesh::new(vertices, polygons, neighbors);
    let heuristic = NavMeshHeuristic { mesh: &mesh };

    // Define points strictly inside the triangles
    let start_pos = [1.0, 0.0, 0.5]; // Inside T0
    let end_pos = [1.0, 0.0, 1.5];   // Inside T1

    println!("Locating start and end polygons...");
    let start_poly = mesh.get_poly_at_pos(start_pos).expect("Start position not on navmesh");
    let goal_poly = mesh.get_poly_at_pos(end_pos).expect("End position not on navmesh");
    
    println!("Start Poly: {}, Goal Poly: {}", start_poly, goal_poly);

    // 1. Run A*
    println!("Running A*...");
    let path_result = astar(
        &mesh,
        &heuristic,
        start_poly,
        goal_poly,
        AStarConfig::default()
    );
    
    println!("A* Path (Polygons): {:?}", path_result.path);
    assert_eq!(path_result.path, vec![0, 1]);

    // 2. Get Portals
    let portals = mesh.get_portals(&path_result.path, start_pos, end_pos);
    println!("Portals: {:?}", portals);
    
    // 3. String Pulling
    println!("Running Funnel Algorithm...");
    let smoothed_path = string_pull(&portals);
    println!("Smoothed Path: {:?}", smoothed_path);
    
    let p0 = smoothed_path[0];
    let p1 = smoothed_path[1];
    
    println!("Start: {:?}", p0);
    println!("End:   {:?}", p1);
    
    assert!((p0[0] - start_pos[0]).abs() < 0.001);
    assert!((p0[2] - start_pos[2]).abs() < 0.001);
    assert!((p1[0] - end_pos[0]).abs() < 0.001);
    assert!((p1[2] - end_pos[2]).abs() < 0.001);
    
    println!("Test Passed!");
}
