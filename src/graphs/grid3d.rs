use crate::traits::Graph;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GridPos3D {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VoxelType {
    Passable(f32),
    Blocked,
}

pub struct Grid3D {
    pub width: usize,
    pub height: usize,
    pub depth: usize,
    pub voxels: Vec<VoxelType>,
}

impl Grid3D {
    pub fn new(width: usize, height: usize, depth: usize) -> Self {
        Self {
            width,
            height,
            depth,
            voxels: vec![VoxelType::Passable(1.0); width * height * depth],
        }
    }

    pub fn set_blocked(&mut self, x: usize, y: usize, z: usize, blocked: bool) {
        if x < self.width && y < self.height && z < self.depth {
            self.voxels[z * self.width * self.height + y * self.width + x] = if blocked {
                VoxelType::Blocked
            } else {
                VoxelType::Passable(1.0)
            };
        }
    }

    pub fn is_blocked(&self, x: i32, y: i32, z: i32) -> bool {
        if x < 0 || y < 0 || z < 0 { return true; }
        let ux = x as usize;
        let uy = y as usize;
        let uz = z as usize;
        if ux >= self.width || uy >= self.height || uz >= self.depth { return true; }
        match self.voxels[uz * self.width * self.height + uy * self.width + ux] {
            VoxelType::Blocked => true,
            _ => false,
        }
    }

    pub fn get_cost(&self, x: i32, y: i32, z: i32) -> f32 {
        if self.is_blocked(x, y, z) { return f32::INFINITY; }
        let idx = (z as usize) * self.width * self.height + (y as usize) * self.width + (x as usize);
        match self.voxels[idx] {
            VoxelType::Passable(c) => c,
            VoxelType::Blocked => f32::INFINITY,
        }
    }
}

impl Graph for Grid3D {
    type Node = GridPos3D;

    fn is_passable(&self, node: &Self::Node) -> bool {
        !self.is_blocked(node.x, node.y, node.z)
    }

    fn neighbors<F>(&self, node: &Self::Node, mut visit: F)
    where
        F: FnMut(Self::Node, f32),
    {
        // 6-connectivity (face neighbors only for simplicity)
        let dirs = [
            (0, 0, 1), (0, 0, -1),
            (0, 1, 0), (0, -1, 0),
            (1, 0, 0), (-1, 0, 0)
        ];

        for (dx, dy, dz) in dirs.iter() {
            let nx = node.x + dx;
            let ny = node.y + dy;
            let nz = node.z + dz;
            
            if !self.is_blocked(nx, ny, nz) {
                visit(GridPos3D { x: nx, y: ny, z: nz }, self.get_cost(nx, ny, nz));
            }
        }
    }
}
