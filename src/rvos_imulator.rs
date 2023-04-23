use nalgebra::Point2;
use ncollide2d::bounding_volume::AABB;
use pi_slotmap::{Key, KeyData};
use pi_spatialtree::{tilemap::TileMap, QuadHelper, QuadTree, Tree};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    agent::{Agent, Line},
    obstacle::{Obstacle, Vertices},
    vector2::Vector2,
};

#[wasm_bindgen]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct ID(pub usize);

impl From<KeyData> for ID {
    fn from(data: KeyData) -> Self {
        ID(data.as_ffi() as usize)
    }
}

unsafe impl Key for ID {
    fn data(&self) -> KeyData {
        KeyData::from_ffi(self.0 as u64)
    }

    fn null() -> Self {
        ID(usize::MAX)
    }

    fn is_null(&self) -> bool {
        self.0 == usize::MAX
    }
}

#[wasm_bindgen]
// #[derive(Clone)]
pub struct RVOSimulator {
    agents_: Vec<Agent>,
    default_agent: Option<Agent>,
    pub global_time: f32,
    obstacles_: Vec<Obstacle>,
    pub time_step: f32,
    tile_map: TileMap<f32, ID, i32>,
    obstacles_map: Tree<ID, QuadHelper<f32>, Vertices, 4>,
}

// let mut tree = QuadTree::new(
//     AABB::new(
//         Point2::new(-1024f32, -1024f32),
//         Point2::new(3072f32, 3072f32),
//     ),
//     max,
//     min,
//     0,
//     0,
//     0,
// );
#[wasm_bindgen]
impl RVOSimulator {
    pub fn default() -> Self {
        let max = nalgebra::Vector2::new(100f32, 100f32);
        let min = max / 100f32;
        Self {
            agents_: Default::default(),
            default_agent: None,
            global_time: Default::default(),
            obstacles_: Vec::with_capacity(500),
            time_step: Default::default(),
            tile_map: TileMap::new(
                AABB::new(
                    Point2::new(-1024f32, -1024f32),
                    Point2::new(3072f32, 3072f32),
                ),
                10,
                10,
            ),
            obstacles_map: QuadTree::new(
                AABB::new(
                    Point2::new(-1024f32, -1024f32),
                    Point2::new(3072f32, 3072f32),
                ),
                max,
                min,
                0,
                0,
                0,
            ),
        }
    }


    pub fn new(
        max_obstacle: usize,
        time_step: f32,
        neighbor_dist: f32,
        max_neighbors: usize,
        time_horizon: f32,
        time_horizon_obst: f32,
        radius: f32,
        max_speed: f32,
        velocity: &Vector2,
    ) -> Self {
        let max = nalgebra::Vector2::new(100f32, 100f32);
        let min = max / 100f32;
        let mut sim_ = Self {
            agents_: vec![],
            default_agent: None,
            global_time: 0.0,
            obstacles_: Vec::with_capacity(max_obstacle),
            time_step: time_step,
            tile_map: TileMap::new(
                AABB::new(
                    Point2::new(-1024f32, -1024f32),
                    Point2::new(3072f32, 3072f32),
                ),
                10,
                10,
            ),
            obstacles_map: QuadTree::new(
                AABB::new(
                    Point2::new(-1024f32, -1024f32),
                    Point2::new(3072f32, 3072f32),
                ),
                max,
                min,
                0,
                0,
                0,
            ),
        };

        let mut agent = Agent::new(&mut sim_);
        agent.max_neighbors = max_neighbors;
        agent.max_speed = max_speed;
        agent.neighbor_dist = neighbor_dist;
        agent.radius_ = radius;
        agent.time_horizon = time_horizon;
        agent.time_horizon_obst = time_horizon_obst;
        agent.velocity_ = *velocity;
        sim_.default_agent = Some(agent);
        sim_
    }

    pub fn add_agent(&mut self, position: &Vector2) -> usize {
        if self.default_agent.is_none() {
            return usize::MAX;
        }

        let mut agent = Agent::new(self);

        agent.position_ = *position;

        let default_agent = self.default_agent.as_ref().unwrap();
        agent.max_neighbors = default_agent.max_neighbors;
        agent.max_speed = default_agent.max_speed;
        agent.neighbor_dist = default_agent.neighbor_dist;
        agent.radius_ = default_agent.radius_;
        agent.time_horizon = default_agent.time_horizon;
        agent.time_horizon_obst = default_agent.time_horizon_obst;
        agent.velocity_ = default_agent.velocity_;
        agent.id_ = ID(self.agents_.len());
        // println!(
        //     "addAgent !!! id: {:?}, ab: {:?}",
        //     agent.id_,
        //     agent.compute_aabb()
        // );
        self.tile_map.add(agent.id_, agent.compute_aabb(), 0);

        // let agent = Box::into_raw(Box::new(agent));
        self.agents_.push(agent);

        return self.agents_.len() - 1;
    }

    pub fn add_agent2(
        &mut self,
        position: &Vector2,
        neighbor_dist: f32,
        max_neighbors: usize,
        time_horizon: f32,
        time_horizon_obst: f32,
        radius: f32,
        max_speed: f32,
        velocity: &Vector2,
    ) -> usize {
        let mut agent = Agent::new(self);

        agent.position_ = *position;
        agent.max_neighbors = max_neighbors;
        agent.max_speed = max_speed;
        agent.neighbor_dist = neighbor_dist;
        agent.radius_ = radius;
        agent.time_horizon = time_horizon;
        agent.time_horizon_obst = time_horizon_obst;
        agent.velocity_ = *velocity;

        agent.id_ = ID(self.agents_.len());

        self.tile_map.add(agent.id_, agent.compute_aabb(), 0);

        // let agent = Box::into_raw(Box::new(agent));
        self.agents_.push(agent);

        return self.agents_.len() - 1;
    }

    pub fn add_obstacle(&mut self, vertices: Vertices) -> usize {
        if vertices.len() < 2 {
            return usize::MAX;
        }

        let obstacle_no = self.obstacles_.len();

        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;

        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for i in 0..vertices.len() {
            let mut obstacle = Obstacle::default();
            // let obstacle2 = unsafe { &mut *obstacle };
            let pos = vertices.get(i);
            obstacle.point_ = pos;

            // if i != 0 {
            //     obstacle.prev_obstacle = self.obstacles_.last_mut().unwrap();
            //     (unsafe { *obstacle.prev_obstacle }).next_obstacle = &mut obstacle;
            // }

            // if i == vertices.len() - 1 {
            //     obstacle.next_obstacle = &mut self.obstacles_[obstacle_no];
            //     (unsafe { *obstacle.next_obstacle }).prev_obstacle = &mut obstacle;
            // }

            let a = if i == vertices.len() - 1 { 0 } else { i + 1 };
            obstacle.unit_dir = Vector2::normalize(&(vertices.get(a) - pos));

            if vertices.len() == 2 {
                obstacle.is_convex = true;
            } else {
                let a = if i == 0 { vertices.len() - 1 } else { i - 1 };
                let b = if i == vertices.len() - 1 { 0 } else { i + 1 };

                obstacle.is_convex =
                    Vector2::left_of(&vertices.get(a), &pos, &vertices.get(b)) >= 0.0;
            }
            let id = self.obstacles_.len();
            obstacle.id_ = id;

            self.obstacles_.push(obstacle);
            println!("obstacle: {:?}", obstacle);
            if pos.x < min_x {
                min_x = pos.x;
            }
            if pos.y < min_y {
                min_y = pos.y;
            }
            if pos.x > max_x {
                max_x = pos.x;
            }
            if pos.y > max_y {
                max_y = pos.y;
            }
        }

        let begin = self.obstacles_.len() - vertices.len();
        let len = self.obstacles_.len();
        for i in begin..len {
            println!("i: {}", i);
            let next_index = if i == len - 1 { begin } else { i + 1 };
            let prev_index = if i == 0 { len - 1 } else { i - 1 };

            self.obstacles_[i].next_obstacle = &mut self.obstacles_[next_index];
            self.obstacles_[i].prev_obstacle = &mut self.obstacles_[prev_index];
        }

        let id = self.obstacles_.len() - vertices.len();
        let aabb = AABB::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y));
        println!("addObstacle !!! id: {:?}, ab: {:?}", id, aabb);
        self.obstacles_map.add(ID(id), aabb, vertices);
        // let mut i = 0;
        // for o in &self.obstacles_ {
        //     println!("obstacle{}: {:?}, ptr: {:?}",i, o, o as *const _);
        //     i += 1;
        // }

        return obstacle_no;
    }

    pub fn do_step(&mut self) {
        // let mut i = 0;
        // for obstacle in &self.obstacles_ {
        //     println!("obstacle{}: {:?}, ptr: {:?}",i,  obstacle, obstacle as *const _);
        //     i += 1
        // }
        self.update_tree();

        for agents in &mut self.agents_ {
            agents.compute_neighbors();
            agents.compute_new_velocity();
        }

        for agents in &mut self.agents_ {
            agents.update();
        }

        self.global_time += self.time_step;
    }

    fn update_tree(&mut self) {
        // unsafe { (&mut *self.tile_map).clear() };

        for agents in &mut self.agents_ {
            // println!(
            //     "updateTree !!! id: {:?}, ab: {:?}",
            //     unsafe { (**agents).id_ },
            //     unsafe { (**agents).compute_aabb() }
            // );
            self.tile_map.update(agents.id_, agents.compute_aabb());
        }
    }

    pub fn get_agent_agent_neighbor(&self, agent_no: usize, neighbor_no: usize) -> usize {
        return self.agents_[agent_no].get_agent_agent_neighbor(neighbor_no);
    }

    pub fn get_agent_max_neighbors(&self, agent_no: usize) -> usize {
        return self.agents_[agent_no].max_neighbors;
    }

    pub fn get_agent_max_speed(&self, agent_no: usize) -> f32 {
        return self.agents_[agent_no].max_speed;
    }

    pub fn get_agent_neighbor_dist(&self, agent_no: usize) -> f32 {
        return self.agents_[agent_no].neighbor_dist;
    }

    pub fn get_agent_num_agent_neighbors(&self, agent_no: usize) -> usize {
        return self.agents_[agent_no].get_agent_num_agent_neighbors();
    }

    pub fn get_agent_num_obstacle_neighbors(&self, agent_no: usize) -> usize {
        return self.agents_[agent_no].get_agent_num_obstacle_neighbors();
    }

    pub fn get_agent_num_orcalines(&self, agent_no: usize) -> usize {
        return self.agents_[agent_no].get_agent_num_orcalines();
    }

    pub fn get_agent_obstacle_neighbor(&self, agent_no: usize, neighbor_no: usize) -> usize {
        return self.agents_[agent_no].get_agent_obstacle_neighbor(neighbor_no);
        // return agents_[agent_no]->obstacle_neighbors[neighborNo].second->id_;
    }

    pub fn get_agent_orcaline(&self, agent_no: usize, line_no: usize) -> Line {
        return self.agents_[agent_no].get_agent_orcaline(line_no);
    }

    pub fn get_agent_position(&self, agent_no: usize) -> Vector2 {
        return self.agents_[agent_no].position_;
    }

    pub fn get_agent_pref_velocity(&self, agent_no: usize) -> Vector2 {
        return self.agents_[agent_no].pref_velocity;
    }

    pub fn get_agent_radius(&self, agent_no: usize) -> f32 {
        return self.agents_[agent_no].radius_;
    }

    pub fn get_agent_time_horizon(&self, agent_no: usize) -> f32 {
        return self.agents_[agent_no].time_horizon;
    }

    pub fn get_agent_time_horizon_obst(&self, agent_no: usize) -> f32 {
        return self.agents_[agent_no].time_horizon_obst;
    }

    pub fn get_agent_velocity(&self, agent_no: usize) -> Vector2 {
        return self.agents_[agent_no].velocity_;
    }

    pub fn get_global_time(&self) -> f32 {
        return self.global_time;
    }

    pub fn get_num_agents(&self) -> usize {
        return self.agents_.len();
    }

    pub fn get_num_obstacle_vertices(&self) -> usize {
        return self.obstacles_.len();
    }

    pub fn get_obstacle_vertex(&self, vertex_no: usize) -> Vector2 {
        return self.obstacles_[vertex_no].point_;
    }

    pub fn get_next_obstacle_vertex_no(&self, vertex_no: usize) -> usize {
        return (unsafe { *(self.obstacles_[vertex_no]).next_obstacle }).id_;
    }

    pub fn get_prev_obstacle_vertex_no(&self, vertex_no: usize) -> usize {
        return (unsafe { *(self.obstacles_[vertex_no]).prev_obstacle }).id_;
    }

    pub fn get_time_step(&self) -> f32 {
        return self.time_step;
    }

    pub fn get_agents(&mut self, agent_no: usize) -> *mut Agent {
        return &mut self.agents_[agent_no];
    }

    pub fn process_obstacles() {
        // kdTree_->buildObstacleTree();
    }

    //  pub fn queryVisibility(point1 : &Vector2, point2:  &Vector2 , radius: f32 ) ->bool
    // {
    // 	// return kdTree_->queryVisibility(point1, point2, radius);
    // }

    pub fn set_agent_defaults(
        &mut self,
        neighbor_dist: f32,
        max_neighbors: usize,
        time_horizon: f32,
        time_horizon_obst: f32,
        radius: f32,
        max_speed: f32,
        velocity: &Vector2,
    ) {
        if self.default_agent.is_none() {
            self.default_agent = Some(Agent::new(self));
        }

        let default_agent = self.default_agent.as_mut().unwrap();
        default_agent.max_neighbors = max_neighbors;
        default_agent.max_speed = max_speed;
        default_agent.neighbor_dist = neighbor_dist;
        default_agent.radius_ = radius;
        default_agent.time_horizon = time_horizon;
        default_agent.time_horizon_obst = time_horizon_obst;
        default_agent.velocity_ = *velocity;
        // println!(
        //     "===== radius_: {}",
        //     unsafe { (&*self.defaultAgent_) }.radius_
        // )
    }

    pub fn set_agent_max_neighbors(&mut self, agent_no: usize, max_neighbors: usize) {
        self.agents_[agent_no].max_neighbors = max_neighbors;
    }

    pub fn set_agent_max_speed(&mut self, agent_no: usize, max_speed: f32) {
        self.agents_[agent_no].max_speed = max_speed;
    }

    pub fn set_agent_neighbor_dist(&mut self, agent_no: usize, neighbor_dist: f32) {
        self.agents_[agent_no].neighbor_dist = neighbor_dist;
    }

    pub fn set_agent_position(&mut self, agent_no: usize, position: &Vector2) {
        self.agents_[agent_no].position_ = *position;
    }

    pub fn set_agent_pref_velocity(&mut self, agent_no: usize, pref_velocity: &Vector2) {
        self.agents_[agent_no].pref_velocity = *pref_velocity;
    }

    pub fn set_agent_radius(&mut self, agent_no: usize, radius: f32) {
        self.agents_[agent_no].radius_ = radius;
    }

    pub fn set_agent_time_horizon(&mut self, agent_no: usize, time_horizon: f32) {
        self.agents_[agent_no].time_horizon = time_horizon;
    }

    pub fn set_agent_time_horizon_obst(&mut self, agent_no: usize, time_horizon_obst: f32) {
        self.agents_[agent_no].time_horizon_obst = time_horizon_obst;
    }

    pub fn set_agent_velocity(&mut self, agent_no: usize, velocity: &Vector2) {
        self.agents_[agent_no].velocity_ = *velocity;
    }

    pub fn set_time_step(&mut self, time_step: f32) {
        self.time_step = time_step;
    }
}

impl RVOSimulator {
    pub fn compute_neighbors_aabb(&mut self, ab: AABB<f32>) -> Vec<usize> {
        let mut r = vec![];
        let tree = &self.tile_map;
        let (_len, iter) = tree.query_iter(&ab);

        for i in iter {
            let mut items = tree.get_tile_iter(i);

            for _ in 0..items.0 {
                if let Some((id, _ab, _)) = items.1.next() {
                    // println!("id: {:?}, ab: {:?}", id, ab);
                    r.push(id.0);
                }
            }
        }
        r
    }

    pub fn compute_obstacle_aabb(&mut self, ab: AABB<f32>) -> Vec<(usize, Vertices)> {
        let mut r = vec![];
        let tree = &self.obstacles_map;
        let mut args = AbQueryArgs::new(ab.clone());
        tree.query(&ab, intersects, &mut args, ab_query_func);
        //assert_eq!(args.result(), [1, 3, 4]);

        for i in args.result {
            r.push((i.0 .0, i.1));
        }
        r
    }

    pub fn get_obstacles(&self, agent_no: usize) -> Obstacle {
        // println!("obstacles: {:?}, agent_no: {}", self.obstacles_.len(), agent_no);
        // if self.global_time > 68.{
            // println!("obstacles: {:?}", self.obstacles_);
            // println!("obstacles{}: {:?}",agent_no,  self.obstacles_[agent_no].clone(),);
        // }
        return self.obstacles_[agent_no].clone();
    }
}

/// quad节点查询函数的范本，aabb是否相交，参数a是查询参数，参数b是quad节点的aabb， 所以最常用的判断是左闭右开
/// 应用方为了功能和性能，应该实现自己需要的quad节点的查询函数， 比如点查询， 球查询， 视锥体查询...
pub fn intersects(a: &AABB<f32>, b: &AABB<f32>) -> bool {
    a.mins.x <= b.maxs.x && a.maxs.x > b.mins.x && a.mins.y <= b.maxs.y && a.maxs.y > b.mins.y
}

/// aabb的查询函数的参数
pub struct AbQueryArgs {
    pub aabb: AABB<f32>,
    pub result: Vec<(ID, Vertices)>,
}
impl AbQueryArgs {
    pub fn new(aabb: AABB<f32>) -> AbQueryArgs {
        AbQueryArgs {
            aabb: aabb,
            result: vec![],
        }
    }
}

/// ab节点的查询函数, 这里只是一个简单范本，使用了quad节点的查询函数intersects
/// 应用方为了功能和性能，应该实现自己需要的ab节点的查询函数， 比如点查询， 球查询-包含或相交， 视锥体查询...
pub fn ab_query_func(arg: &mut AbQueryArgs, id: ID, aabb: &AABB<f32>, bind: &Vertices) {
    // println!("ab_query_func: id: {}, bind:{:?}, arg: {:?}", id, bind, arg.result);
    if intersects(&arg.aabb, aabb) {
        arg.result.push((id, bind.clone()));
    }
}
