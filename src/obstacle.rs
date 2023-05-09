use pi_slotmap::DefaultKey;

use crate::{ vector2::Vector2};

#[derive(Clone, Copy, Debug)]
pub struct Obstacle {
    pub is_convex: bool,
    pub next_obstacle: DefaultKey,
    pub point_: Vector2,
    pub prev_obstacle: DefaultKey,
    pub unit_dir: Vector2,
    pub id_: DefaultKey,
}

impl Obstacle {
    pub fn default() -> Self {
        Self {
            is_convex: false,
            next_obstacle: Default::default(),
            point_: Default::default(),
            prev_obstacle: Default::default(),
            unit_dir: Default::default(),
            id_: Default::default(),
        }
    }
}

