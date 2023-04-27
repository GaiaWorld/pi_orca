use crate::{
    agent::Agent,
    obstacle::Obstacle,
    util::{ab_query_func, intersects, AbQueryArgs, Line, Vertices, ID},
    vector2::Vector2,
};

use nalgebra::Point2;
use parry2d::bounding_volume::Aabb as AABB;
use pi_slotmap::SlotMap;
use pi_spatial::{tilemap::TileMap, QuadHelper, QuadTree, Tree};
use wasm_bindgen::prelude::wasm_bindgen;

#[wasm_bindgen]
pub struct RVOSimulator {
    agents_: SlotMap<ID, Agent>,
    default_agent: Option<Agent>,
    pub global_time: f32,
    obstacles_: SlotMap<ID, Obstacle>,
    pub time_step: f32,
    tile_map: TileMap<ID, i32>,
    obstacles_map: Tree<ID, QuadHelper, Vertices, 4>,
}

#[wasm_bindgen]
impl RVOSimulator {
    pub fn default(_max_obstacle: usize) -> Self {
        let max = nalgebra::Vector2::new(100f32, 100f32);
        let min = max / 100f32;
        Self {
            agents_: Default::default(),
            default_agent: None,
            global_time: Default::default(),
            obstacles_: Default::default(),
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
        _max_obstacle: usize,
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
            agents_: Default::default(),
            default_agent: None,
            global_time: 0.0,
            obstacles_: Default::default(),
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

    pub fn add_agent(&mut self, position: &Vector2) -> f64 {
        if self.default_agent.is_none() {
            return f64::MAX;
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
        // agent.id_ = id;
        // println!(
        //     "addAgent !!! id: {:?}, ab: {:?}",
        //     agent.id_,
        //     agent.compute_aabb()
        // );
        let ab: AABB = agent.compute_aabb();
        let id = self.agents_.insert(agent);
        if let Some(a) = self.agents_.get_mut(id) {
            a.id_ = id;
        }
        self.tile_map.add(id, ab, 0);

        // let agent = Box::into_raw(Box::new(agent));

        return id.0;
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
    ) -> f64 {
        let mut agent = Agent::new(self);

        agent.position_ = *position;
        agent.max_neighbors = max_neighbors;
        agent.max_speed = max_speed;
        agent.neighbor_dist = neighbor_dist;
        agent.radius_ = radius;
        agent.time_horizon = time_horizon;
        agent.time_horizon_obst = time_horizon_obst;
        agent.velocity_ = *velocity;

        let ab: AABB = agent.compute_aabb();
        let id = self.agents_.insert(agent);
        if let Some(a) = self.agents_.get_mut(id) {
            a.id_ = id;
        }

        self.tile_map.add(id, ab, 0);

        return id.0;
    }

    pub fn remove_agent(&mut self, id: f64) -> bool {
        if let Some(_) = self.tile_map.remove(ID(id)) {
            if let Some(_) = self.agents_.remove(ID(id)) {
                return true;
            }
        }

        false
    }

    /**
     * @brief 为模拟添加新障碍。
     * @param[in] vertices 逆时针顺序排列的多边形障碍物的顶点列表。
     * @return 障碍物第一个顶点的编号，当顶点数小于2时为返回usize::MAX。
     * @note 要添加“负面”障碍，例如环境周围的边界多边形，顶点应按顺时针顺序列出。
     */
    pub fn add_obstacle(&mut self, vertices: Vertices) -> f64 {
        if vertices.len() < 2 {
            return f64::MAX;
        }

        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;

        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        let mut ids = vec![];
        for i in 0..vertices.len() {
            let mut obstacle = Obstacle::default();
            let pos = vertices.get(i);
            obstacle.point_ = pos;

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

            let id = self.obstacles_.insert(obstacle);
            if let Some(o) = self.obstacles_.get_mut(id) {
                o.id_ = id;
            }
            ids.push(id);
            // println!("obstacle: {:?}", obstacle);
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

        let len = ids.len();
        for i in 0..len {
            // println!("i: {}", i);
            let next_index = if i == len - 1 { 0 } else { i + 1 };
            let prev_index = if i == 0 { len - 1 } else { i - 1 };

            if let Some(o) = self.obstacles_.get_mut(ids[i]) {
                o.next_obstacle = ids[next_index];
                o.prev_obstacle = ids[prev_index];
            }
        }

        let id = ids[0];
        let aabb = AABB::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y));
        self.obstacles_map.add(id, aabb, vertices);

        return id.0;
    }

    pub fn remove_obstacle(&mut self, id: f64) -> bool {
        let mut key = ID(id);

        let mut temp = 0;
        loop {
            match self.obstacles_.remove(key) {
                Some(o) => key = o.next_obstacle,
                None => break,
            };
            temp += 1;
        }

        if let Some((_ab, bind)) = self.obstacles_map.remove(key) {
            if temp == bind.len() {
                return true;
            }
        }

        false
    }

    pub fn do_step(&mut self) {
        self.update_tree();

        for (_id, agents) in &mut self.agents_ {
            agents.compute_neighbors();
            agents.compute_new_velocity();
        }

        for (_id, agents) in &mut self.agents_ {
            agents.update();
        }

        self.global_time += self.time_step;
    }

    fn update_tree(&mut self) {
        for (id, agents) in &mut self.agents_ {
            self.tile_map.update(id, agents.compute_aabb());
        }
    }

    /**
    * @brief     为添加的任何新代理设置默认属性。
    * @param[in] neighborDist    新代理在导航中考虑的默认最大中心点到中心点到其他代理的最大距离。
                                 这个数字越大，模拟的运行时间就越长。
                                 如果数字太低，模拟将不安全。
                                 必须是非负数。
    * @param[in] maxNeighbors    新代理在导航中考虑的默认最大其他代理数。
                                 这个数字越大，模拟的运行时间越长。
                                 如果数字太低，模拟将不安全。
    * @param[in] timeHorizon     模拟计算的新代理速度相对于其他代理安全的默认最小时间量。
                                 这个数字越大，代理就会越快响应其他代理的存在，但代理在选择速度方面的自由度就越小。
                                 必须是正数。
    * @param[in] timeHorizonObst 通过模拟计算的新代理的速度相对于障碍物是安全的默认最小时间量。
                                 这个数字越大，代理越快响应障碍的存在，但代理的自由度越低 选择它的速度。
                                 必须是正数。
    * @param[in] radius          新代理的默认半径。 必须是非负数。
    * @param[in] maxSpeed        新代理的默认最大速度。 必须是非负数。
    */
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
    }

    pub fn get_agent_agent_neighbor(&self, agent_no: f64, neighbor_no: usize) -> Option<f64> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.get_agent_agent_neighbor(neighbor_no));
        }
        None
    }

    pub fn get_agent_max_neighbors(&self, agent_no: f64) -> Option<usize> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.max_neighbors);
        }
        None
    }

    pub fn get_agent_max_speed(&self, agent_no: f64) -> Option<f32> {
        // return self.agents_[agent_no].max_speed;
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.max_speed);
        }
        None
    }

    pub fn get_agent_neighbor_dist(&self, agent_no: f64) -> Option<f32> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.neighbor_dist);
        }
        None
    }

    pub fn get_agent_num_agent_neighbors(&self, agent_no: f64) -> Option<usize> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.get_agent_num_agent_neighbors());
        }
        None
    }

    pub fn get_agent_num_obstacle_neighbors(&self, agent_no: f64) -> Option<usize> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.get_agent_num_obstacle_neighbors());
        }
        None
    }

    pub fn get_agent_num_orcalines(&self, agent_no: f64) -> Option<usize> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.get_agent_num_orcalines());
        }
        None
    }

    pub fn get_agent_obstacle_neighbor(&self, agent_no: f64, neighbor_no: f64) -> Option<f64> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.get_agent_obstacle_neighbor(neighbor_no as usize));
        }
        None
    }

    pub fn get_agent_position(&self, agent_no: f64) -> Option<Vector2> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.position_);
        }
        None
    }

    pub fn get_agent_pref_velocity(&self, agent_no: f64) -> Option<Vector2> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.pref_velocity);
        }
        None
    }

    pub fn get_agent_radius(&self, agent_no: f64) -> Option<f32> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.radius_);
        }
        None
    }

    pub fn get_agent_time_horizon(&self, agent_no: f64) -> Option<f32> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.time_horizon);
        }
        None
    }

    pub fn get_agent_time_horizon_obst(&self, agent_no: f64) -> Option<f32> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.time_horizon);
        }
        None
    }

    pub fn get_agent_velocity(&self, agent_no: f64) -> Option<Vector2> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.velocity_);
        }
        None
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

    pub fn get_obstacle_vertex(&self, vertex_no: f64) -> Option<Vector2> {
        if let Some(v) = self.obstacles_.get(ID(vertex_no)) {
            return Some(v.point_);
        }
        None
    }

    pub fn get_next_obstacle_vertex_no(&self, vertex_no: f64) -> Option<f64> {
        if let Some(v) = self.obstacles_.get(ID(vertex_no)) {
            if let Some(next_obstacle) = self.obstacles_.get(v.next_obstacle) {
                return Some(next_obstacle.id_.0);
            }
        }
        None
    }

    pub fn get_prev_obstacle_vertex_no(&self, vertex_no: f64) -> Option<f64> {
        if let Some(v) = self.obstacles_.get(ID(vertex_no)) {
            if let Some(prev_obstacle) = self.obstacles_.get(v.prev_obstacle) {
                return Some(prev_obstacle.id_.0);
            }
        }
        None
    }

    pub fn get_time_step(&self) -> f32 {
        return self.time_step;
    }

    pub fn set_agent_max_neighbors(&mut self, agent_no: f64, max_neighbors: usize) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.max_neighbors = max_neighbors;
            return true;
        }
        false
    }

    pub fn set_agent_max_speed(&mut self, agent_no: f64, max_speed: f32) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.max_speed = max_speed;
            return true;
        }
        false
    }

    pub fn set_agent_neighbor_dist(&mut self, agent_no: f64, neighbor_dist: f32) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.neighbor_dist = neighbor_dist;
            return true;
        }
        false
    }

    pub fn set_agent_position(&mut self, agent_no: f64, position: &Vector2) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.position_ = *position;
            return true;
        }
        false
    }

    pub fn set_agent_pref_velocity(&mut self, agent_no: f64, pref_velocity: &Vector2) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.pref_velocity = *pref_velocity;
            return true;
        }
        false
    }

    pub fn set_agent_radius(&mut self, agent_no: f64, radius: f32) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.radius_ = radius;
            return true;
        }
        false
    }

    pub fn set_agent_time_horizon(&mut self, agent_no: f64, time_horizon: f32) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.time_horizon = time_horizon;
            return true;
        }
        false
    }

    pub fn set_agent_time_horizon_obst(&mut self, agent_no: f64, time_horizon_obst: f32) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.time_horizon_obst = time_horizon_obst;
            return true;
        }
        false
    }

    pub fn set_agent_velocity(&mut self, agent_no: f64, velocity: &Vector2) -> bool {
        if let Some(agent) = self.agents_.get_mut(ID(agent_no)) {
            agent.velocity_ = *velocity;
            return true;
        }
        false
    }

    pub fn set_time_step(&mut self, time_step: f32) {
        self.time_step = time_step;
    }
}

impl RVOSimulator {
    /**
     * @brief     计算代理邻居。
     * @param[in] ab   当前代理的AABB
     * @return         返回邻居代理的ID集合
     */
    pub fn compute_neighbors_aabb(&mut self, ab: AABB) -> Vec<ID> {
        let mut r = vec![];
        let tree = &self.tile_map;
        let (_len, iter) = tree.query_iter(&ab);

        for i in iter {
            let mut items = tree.get_tile_iter(i);

            for _ in 0..items.0 {
                if let Some((id, _ab, _)) = items.1.next() {
                    // println!("id: {:?}, ab: {:?}", id, ab);
                    r.push(id);
                }
            }
        }
        r
    }

    /**
     * @brief     计算障碍物邻居
     * @param[in] ab   当前代理的AABB
     * @return         返回障碍物的数据集合
     */
    pub fn compute_obstacle_aabb(&mut self, ab: AABB) -> Vec<(ID, Vertices)> {
        let mut r = vec![];
        let tree = &self.obstacles_map;
        let mut args = AbQueryArgs::new(ab.clone());
        tree.query(&ab, intersects, &mut args, ab_query_func);
        //assert_eq!(args.result(), [1, 3, 4]);

        for i in args.result {
            r.push((i.0, i.1));
        }
        r
    }

    pub fn get_obstacle(&self, obstacle_no: ID) -> Option<*const Obstacle> {
        if let Some(v) = self.obstacles_.get(obstacle_no) {
            return Some(v);
        }
        None
    }

    pub fn get_agent(&mut self, agent_no: ID) -> Option<*const Agent> {
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v);
        }
        None
    }

    pub fn get_agent_orcaline(&self, agent_no: f64, line_no: usize) -> Option<Line> {
        if let Some(v) = self.agents_.get(ID(agent_no)) {
            return Some(v.get_agent_orcaline(line_no));
        }
        None
    }
}
