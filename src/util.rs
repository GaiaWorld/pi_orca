use crate::vector2::Vector2;
use parry2d::bounding_volume::Aabb as AABB;
use pi_slotmap::DefaultKey;
use serde::Serialize;

/// quad节点查询函数的范本，aabb是否相交，参数a是查询参数，参数b是quad节点的aabb， 所以最常用的判断是左闭右开
/// 应用方为了功能和性能，应该实现自己需要的quad节点的查询函数， 比如点查询， 球查询， 视锥体查询...
pub fn intersects(a: &AABB, b: &AABB) -> bool {
    a.mins.x <= b.maxs.x && a.maxs.x > b.mins.x && a.mins.y <= b.maxs.y && a.maxs.y > b.mins.y
}

/// aabb的查询函数的参数
pub struct AbQueryArgs {
    pub aabb: AABB,
    pub result: Vec<(DefaultKey, Vec<Vector2>)>,
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
pub fn ab_query_func(arg: &mut AbQueryArgs, id: DefaultKey, aabb: &AABB, bind: &Vec<Vector2>) {
    // println!("ab_query_func: id: {}, bind:{:?}, arg: {:?}", id, bind, arg.result);
    if intersects(&arg.aabb, aabb) {
        arg.result.push((id, bind.clone()));
    }
}

#[derive(Default, Clone, Copy, Debug, Serialize)]
pub struct Line {
    /**
     * \brief     有向线上的一个点。
     */
    pub point: Vector2,

    /**
     * \brief     定向线的方向。
     */
    pub direction: Vector2,
}
