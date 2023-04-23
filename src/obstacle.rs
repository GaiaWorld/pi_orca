use wasm_bindgen::prelude::wasm_bindgen;

use crate::vector2::Vector2;

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct Obstacle {
    pub is_convex: bool,
    pub next_obstacle: *mut Obstacle,
    pub point_: Vector2,
    pub prev_obstacle: *mut Obstacle,
    pub unit_dir: Vector2,
    pub id_: usize,
}

#[wasm_bindgen]
impl Obstacle {
    pub fn default() -> Self {
        Self {
            is_convex: false,
            next_obstacle: std::ptr::null_mut(),
            point_: Default::default(),
            prev_obstacle: std::ptr::null_mut(),
            unit_dir: Default::default(),
            id_: 0,
        }
    }
}

#[wasm_bindgen]
#[derive(Clone, Default, Debug)]
pub struct Vertices(Vec<Vector2>);

#[wasm_bindgen]
impl Vertices {
    pub fn new() -> Self {
        Self(Vec::new())
    }
    
    pub fn add(&mut self, vertex: Vector2) {
        self.0.push(vertex);
    }

    pub fn get(&self, index: usize) -> Vector2 {
        self.0[index]
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }
}