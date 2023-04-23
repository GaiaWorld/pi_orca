use std::iter::Rev;
use std::vec::IntoIter;

use pi_path_finding::{make_neighbors, AStar as AStarInner, PathFilterIter as PathFilterIterInner};
use pi_path_finding::{Entry, NodeIndex as NodeIndexInner, TileMap as TileMapInner};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::vector2::Vector2;

// 瓦片内的障碍

#[wasm_bindgen]
#[derive(Debug, Clone)]
pub enum TileObstacle {
    Right = 1,
    Down = 2,
    Center = 4,
}

#[wasm_bindgen]
#[derive(Debug)]
pub struct NodeIndex {
    inner: NodeIndexInner,
}

#[wasm_bindgen]
impl NodeIndex {
    pub fn new(index: usize) -> Self {
        Self {
            inner: NodeIndexInner(index),
        }
    }

    pub fn index(&self) -> usize {
        self.inner.0
    }
}

#[wasm_bindgen]
pub struct TileMap {
    inner: TileMapInner,
}

#[wasm_bindgen]
pub struct ResultPath {
    res: Rev<IntoIter<Vector2>>,
}

#[wasm_bindgen]
impl ResultPath {
    pub fn next(&mut self) -> Option<Vector2> {
        self.res.next()
    }

}

#[wasm_bindgen]
impl TileMap {
    pub fn new(row: usize, column: usize) -> Self {
        Self {
            inner: TileMapInner::new(row, column, 100, 144),
        }
    }

    pub fn set_obstacle(&mut self, index: NodeIndex, obstacle: TileObstacle) {
        let pos_type = match obstacle {
            TileObstacle::Right => 1,
            TileObstacle::Down => 2,
            TileObstacle::Center => 4,
        };

        self.inner.set_node_obstacle(index.inner, pos_type);
    }
}

#[wasm_bindgen]
pub struct AStar {
    inner: AStarInner<usize, Entry<usize>>,
}

#[wasm_bindgen]
impl AStar {
    pub fn new(row: usize, column: usize, node_number: usize) -> Self {
        Self {
            inner: AStarInner::with_capacity(row * column, node_number),
        }
    }

    pub fn find_path(
        &mut self,
        tile_map: &mut TileMap,
        max_number: usize,
        start: NodeIndex,
        end: NodeIndex,
    ) -> Option<NodeIndex> {
        let result = self.inner.find(
            start.inner,
            end.inner,
            max_number,
            &mut tile_map.inner,
            make_neighbors,
        );

        match result {
            pi_path_finding::AStarResult::Found => return Some(end),
            pi_path_finding::AStarResult::NotFound => return None,
            pi_path_finding::AStarResult::LimitNotFound(index) => {
                return Some(NodeIndex { inner: index })
            }
        };
    }

    pub fn result(&self, node: NodeIndex, column: usize) -> ResultPath {
        let mut res = vec![];

        for item in PathFilterIterInner::new(self.inner.result_iter(node.inner), column) {
            println!(" x: {}, y: {}", (item.0 % column), (item.0 / column));
            res.push(Vector2 {
                x: (item.0 % column) as f32,
                y: (item.0 / column) as f32,
            });
        }
        let r = res.into_iter().rev();

        ResultPath { res: r }
    }
}
