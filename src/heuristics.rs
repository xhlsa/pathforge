use crate::traits::Heuristic;

pub trait Position {
    fn x(&self) -> f32;
    fn y(&self) -> f32;
    fn z(&self) -> f32 { 0.0 }  // Optional for 2D
}

#[derive(Clone, Copy, Debug)]
pub struct Manhattan;

#[derive(Clone, Copy, Debug)]
pub struct Euclidean;

#[derive(Clone, Copy, Debug)]
pub struct Diagonal {
    pub cardinal_cost: f32,  // typically 1.0
    pub diagonal_cost: f32,  // typically 1.414 or 1.0
}

impl Default for Diagonal {
    fn default() -> Self {
        Self {
            cardinal_cost: 1.0,
            diagonal_cost: std::f32::consts::SQRT_2,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Zero;  // For Dijkstra behavior

impl<P: Position> Heuristic<P> for Manhattan {
    fn estimate(&self, from: &P, to: &P) -> f32 {
        (from.x() - to.x()).abs() + (from.y() - to.y()).abs() + (from.z() - to.z()).abs()
    }
}

impl<P: Position> Heuristic<P> for Euclidean {
    fn estimate(&self, from: &P, to: &P) -> f32 {
        let dx = from.x() - to.x();
        let dy = from.y() - to.y();
        let dz = from.z() - to.z();
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

impl<P: Position> Heuristic<P> for Diagonal {
    fn estimate(&self, from: &P, to: &P) -> f32 {
        let dx = (from.x() - to.x()).abs();
        let dy = (from.y() - to.y()).abs();
        
        // For 3D, this is an approximation or we treat Z as cardinal
        let dz = (from.z() - to.z()).abs();

        let min_d = dx.min(dy);
        let max_d = dx.max(dy);
        
        // 2D Diagonal logic + Z as cardinal
        (self.cardinal_cost * (max_d - min_d)) + (self.diagonal_cost * min_d) + (dz * self.cardinal_cost)
    }
}

impl<P> Heuristic<P> for Zero {
    fn estimate(&self, _from: &P, _to: &P) -> f32 {
        0.0
    }
}
