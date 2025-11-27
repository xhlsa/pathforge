use crate::traits::Graph;

pub enum SmoothingMethod {
    None,
    RemoveRedundant, // String pulling / Line-of-sight shortening
}

pub fn smooth_path<G: Graph>(
    graph: &G,
    path: &[G::Node],
    method: SmoothingMethod,
) -> Vec<G::Node> {
    if path.len() < 3 { return path.to_vec(); }
    
    match method {
        SmoothingMethod::None => path.to_vec(),
        SmoothingMethod::RemoveRedundant => {
             let mut smooth = vec![path[0].clone()];
             let mut current_idx = 0;
             
             while current_idx < path.len() - 1 {
                 let mut next_idx = current_idx + 1;
                 // Look ahead as far as possible
                 // We scan from the END backwards to find the furthest visible node
                 // optimization: check reverse
                 for i in (current_idx + 2..path.len()).rev() {
                     if graph.can_traverse(&path[current_idx], &path[i]) {
                         next_idx = i;
                         break;
                     }
                 }
                 smooth.push(path[next_idx].clone());
                 current_idx = next_idx;
             }
             
             smooth
        }
    }
}
