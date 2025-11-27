use crate::traits::Graph;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GridPos {
    pub x: i32,
    pub y: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CellType {
    Passable(f32),   // with movement cost multiplier
    Blocked,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiagonalMode {
    Never,
    Always,
    IfNoObstacle,    // At least one cardinal neighbor must be open
    OnlyIfBothOpen,  // Both adjacent cardinals must be open (strict corner cutting)
}

pub struct Grid2D {
    pub width: usize,
    pub height: usize,
    pub cells: Vec<CellType>,
    pub diagonal_movement: DiagonalMode,
}

impl Grid2D {
    pub fn new(width: usize, height: usize, diagonal_movement: DiagonalMode) -> Self {
        Self {
            width,
            height,
            cells: vec![CellType::Passable(1.0); width * height],
            diagonal_movement,
        }
    }

    pub fn set_blocked(&mut self, x: usize, y: usize, blocked: bool) {
        if x < self.width && y < self.height {
            self.cells[y * self.width + x] = if blocked {
                CellType::Blocked
            } else {
                CellType::Passable(1.0)
            };
        }
    }

    pub fn set_cost(&mut self, x: usize, y: usize, cost: f32) {
        if x < self.width && y < self.height {
            self.cells[y * self.width + x] = CellType::Passable(cost);
        }
    }

    pub fn is_blocked(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 { return true; }
        let ux = x as usize;
        let uy = y as usize;
        if ux >= self.width || uy >= self.height { return true; }
        match self.cells[uy * self.width + ux] {
            CellType::Blocked => true,
            _ => false,
        }
    }
    
    pub fn get_cost(&self, x: i32, y: i32) -> f32 {
        if x < 0 || y < 0 { return f32::INFINITY; }
        let ux = x as usize;
        let uy = y as usize;
        if ux >= self.width || uy >= self.height { return f32::INFINITY; }
        match self.cells[uy * self.width + ux] {
            CellType::Passable(c) => c,
            CellType::Blocked => f32::INFINITY,
        }
    }
    
    pub fn set_region_blocked(&mut self, rect: (usize, usize, usize, usize), blocked: bool) {
        let (rx, ry, rw, rh) = rect;
        for y in ry..(ry + rh) {
            for x in rx..(rx + rw) {
                self.set_blocked(x, y, blocked);
            }
        }
    }
    
    pub fn clear(&mut self) {
        for c in self.cells.iter_mut() {
            *c = CellType::Passable(1.0);
        }
    }
}

use crate::heuristics::Position;
impl Position for GridPos {
    fn x(&self) -> f32 { self.x as f32 }
    fn y(&self) -> f32 { self.y as f32 }
}

impl Graph for Grid2D {
    type Node = GridPos;

    fn is_passable(&self, node: &Self::Node) -> bool {
        !self.is_blocked(node.x, node.y)
    }

    fn neighbors<F>(&self, node: &Self::Node, mut visit: F)
    where
        F: FnMut(Self::Node, f32),
    {
        // Cardinals
        let dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)];
        for (dx, dy) in dirs.iter() {
            let nx = node.x + dx;
            let ny = node.y + dy;
            if !self.is_blocked(nx, ny) {
                visit(GridPos { x: nx, y: ny }, self.get_cost(nx, ny));
            }
        }
        
        // Diagonals
        if self.diagonal_movement != DiagonalMode::Never {
            let diag_dirs = [(1, 1), (1, -1), (-1, 1), (-1, -1)];
            let diag_cost_mult = std::f32::consts::SQRT_2; // Standard 1.414
            
            for (dx, dy) in diag_dirs.iter() {
                let nx = node.x + dx;
                let ny = node.y + dy;
                
                if !self.is_blocked(nx, ny) {
                    let cost = self.get_cost(nx, ny) * diag_cost_mult;
                    
                    // Check diagonal rules
                    let c1_blocked = self.is_blocked(node.x + dx, node.y);
                    let c2_blocked = self.is_blocked(node.x, node.y + dy);
                    
                    let allowed = match self.diagonal_movement {
                        DiagonalMode::Never => false,
                        DiagonalMode::Always => true,
                        DiagonalMode::IfNoObstacle => !c1_blocked || !c2_blocked,
                        DiagonalMode::OnlyIfBothOpen => !c1_blocked && !c2_blocked,
                    };
                    
                    if allowed {
                        visit(GridPos { x: nx, y: ny }, cost);
                    }
                }
            }
        }
    }

    fn can_traverse(&self, from: &Self::Node, to: &Self::Node) -> bool {
        let x0 = from.x;
        let y0 = from.y;
        let x1 = to.x;
        let y1 = to.y;
        
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;
        
        let mut x = x0;
        let mut y = y0;
        
        while x != x1 || y != y1 {
            if self.is_blocked(x, y) { return false; }
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x += sx;
            }
            if e2 < dx {
                err += dx;
                y += sy;
            }
        }
        // Check destination
        if self.is_blocked(x, y) { return false; }
        
        true
    }
}
