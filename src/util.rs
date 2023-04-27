use crate::vector2::Vector2;
use core::cmp::Ordering;
use core::hash::Hash;
use parry2d::bounding_volume::Aabb as AABB;
use pi_slotmap::{Key, KeyData};
use std::hash::Hasher;
use wasm_bindgen::prelude::wasm_bindgen;

/// quad节点查询函数的范本，aabb是否相交，参数a是查询参数，参数b是quad节点的aabb， 所以最常用的判断是左闭右开
/// 应用方为了功能和性能，应该实现自己需要的quad节点的查询函数， 比如点查询， 球查询， 视锥体查询...
pub fn intersects(a: &AABB, b: &AABB) -> bool {
    a.mins.x <= b.maxs.x && a.maxs.x > b.mins.x && a.mins.y <= b.maxs.y && a.maxs.y > b.mins.y
}

/// aabb的查询函数的参数
pub struct AbQueryArgs {
    pub aabb: AABB,
    pub result: Vec<(ID, Vertices)>,
}
impl AbQueryArgs {
    pub fn new(aabb: AABB) -> AbQueryArgs {
        AbQueryArgs {
            aabb: aabb,
            result: vec![],
        }
    }
}

/// ab节点的查询函数, 这里只是一个简单范本，使用了quad节点的查询函数intersects
/// 应用方为了功能和性能，应该实现自己需要的ab节点的查询函数， 比如点查询， 球查询-包含或相交， 视锥体查询...
pub fn ab_query_func(arg: &mut AbQueryArgs, id: ID, aabb: &AABB, bind: &Vertices) {
    // println!("ab_query_func: id: {}, bind:{:?}, arg: {:?}", id, bind, arg.result);
    if intersects(&arg.aabb, aabb) {
        arg.result.push((id, bind.clone()));
    }
}

#[derive(Default, Clone, Copy, Debug)]
pub struct Line {
    /**
     * \brief     A point on the directed line.
     */
    pub point: Vector2,

    /**
     * \brief     The direction of the directed line.
     */
    pub direction: Vector2,
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

#[wasm_bindgen]
#[derive(Debug, Clone, Copy, Default)]
pub struct ID(pub f64);

impl Ord for ID {
    fn cmp(&self, other: &Self) -> Ordering {
        let a = unsafe { std::mem::transmute::<f64, u64>(self.0) };
        let b = unsafe { std::mem::transmute::<f64, u64>(other.0) };
        a.cmp(&b)
    }
}

impl PartialOrd for ID {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let a = unsafe { std::mem::transmute::<f64, u64>(self.0) };
        let b = unsafe { std::mem::transmute::<f64, u64>(other.0) };
        a.partial_cmp(&b)
    }
}

impl Hash for ID {
    fn hash<H: Hasher>(&self, state: &mut H) {
        let a = unsafe { std::mem::transmute::<f64, u64>(self.0) };
        a.hash(state)
    }
}

impl Eq for ID {}

impl PartialEq for ID {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl From<KeyData> for ID {
    fn from(data: KeyData) -> Self {
        ID(unsafe { std::mem::transmute(data.as_ffi()) })
    }
}

unsafe impl Key for ID {
    fn data(&self) -> KeyData {
        KeyData::from_ffi(unsafe { std::mem::transmute(self.0) })
    }

    fn null() -> Self {
        ID(f64::MAX)
    }

    fn is_null(&self) -> bool {
        self.0 == f64::MAX
    }
}
