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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graphs::grid2d::{Grid2D, GridPos, DiagonalMode};

    #[test]
    fn smoothes_clear_line() {
        let grid = Grid2D::new(8, 8, DiagonalMode::Always);
        let raw = vec![
            GridPos { x: 0, y: 0 },
            GridPos { x: 1, y: 0 },
            GridPos { x: 2, y: 1 },
            GridPos { x: 3, y: 2 },
            GridPos { x: 4, y: 3 },
        ];

        let smoothed = smooth_path(&grid, &raw, SmoothingMethod::RemoveRedundant);
        assert_eq!(smoothed, vec![GridPos { x: 0, y: 0 }, GridPos { x: 4, y: 3 }]);
    }

    #[test]
    fn preserves_obstacle_bends() {
        let mut grid = Grid2D::new(5, 5, DiagonalMode::Always);
        // Wall along y=1 forces path to dip down to y=2
        for x in 0..5 {
            grid.set_blocked(x, 1, true);
        }
        let raw = vec![
            GridPos { x: 0, y: 0 },
            GridPos { x: 0, y: 2 },
            GridPos { x: 1, y: 2 },
            GridPos { x: 2, y: 2 },
            GridPos { x: 3, y: 2 },
            GridPos { x: 4, y: 2 },
        ];

        let smoothed = smooth_path(&grid, &raw, SmoothingMethod::RemoveRedundant);
        assert!(
            smoothed.len() > 2,
            "Obstacle should prevent full straightening; got {:?}",
            smoothed
        );
    }
}
