type Vec3 = [f32; 3];

/// A portal is just the two vertices of the edge we are passing through.
/// Relative to the path direction.
#[derive(Debug, Clone, Copy)]
pub struct Portal {
    pub left: Vec3,
    pub right: Vec3,
}

/// Signed area of a triangle in the XZ plane. 
/// Positive = Left of vector, Negative = Right of vector.
#[inline(always)]
fn tri_area_2d(a: &Vec3, b: &Vec3, c: &Vec3) -> f32 {
    let ax = b[0] - a[0];
    let az = b[2] - a[2];
    let bx = c[0] - a[0];
    let bz = c[2] - a[2];
    bx * az - ax * bz
}

/// The String Pulling (Funnel) Algorithm
/// 
/// `portals`: The sequence of edges including start (p, p) and end (goal, goal).
pub fn string_pull(portals: &[Portal]) -> Vec<Vec3> {
    let mut path = Vec::with_capacity(portals.len());
    if portals.is_empty() {
        return path;
    }

    // The "Apex" is the pivot point of the funnel (usually the last corner we turned).
    let mut apex = portals[0].left; 
    let mut portal_left = portals[0].left;
    let mut portal_right = portals[0].right;

    // Indices to keep track of where the funnel sides are in the list
    // so we don't process old portals.
    let mut left_index = 0;
    let mut right_index = 0;

    // Add start point
    path.push(apex);

    let mut i = 1;
    while i < portals.len() {
        let left = portals[i].left;
        let right = portals[i].right;

        // Update Right Leg
        // Check if the new right vertex tightens the funnel (is to the left of the current right leg)
        // logic: if tri_area(apex, portal_right, right) <= 0.0
        if tri_area_2d(&apex, &portal_right, &right) <= 0.0 {
            // It tightens. Now check if it crosses over the Left Leg.
            if apex == portal_right || tri_area_2d(&apex, &portal_left, &right) > 0.0 {
                // Tighten the funnel
                portal_right = right;
                right_index = i;
            } else {
                // CROSSOVER! The right leg crossed the left leg.
                // We must turn a corner around the LEFT leg.
                apex = portal_left;
                path.push(apex);
                
                // Reset the funnel to the new apex
                portal_left = apex;
                portal_right = apex;
                
                // Restart scan from the portal that formed the corner
                i = left_index + 1;
                left_index = i;
                right_index = i;
                continue;
            }
        }

        // Update Left Leg
        // Check if the new left vertex tightens the funnel (is to the right of the current left leg)
        if tri_area_2d(&apex, &portal_left, &left) >= 0.0 {
            // It tightens. Now check if it crosses over the Right Leg.
            if apex == portal_left || tri_area_2d(&apex, &portal_right, &left) < 0.0 {
                // Tighten the funnel
                portal_left = left;
                left_index = i;
            } else {
                // CROSSOVER! The left leg crossed the right leg.
                // We must turn a corner around the RIGHT leg.
                apex = portal_right;
                path.push(apex);
                
                // Reset the funnel
                portal_left = apex;
                portal_right = apex;
                
                // Restart scan
                i = right_index + 1;
                left_index = i;
                right_index = i;
                continue;
            }
        }
        
        i += 1;
    }

    // Add the final goal point if it wasn't the last apex
    if let Some(last) = portals.last() {
        // usually last.left == last.right == goal
        if path.last() != Some(&last.left) {
             path.push(last.left);
        }
    }

    path
}
