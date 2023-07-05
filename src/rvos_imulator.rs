use crate::{
    agent::Agent,
    obstacle::Obstacle,
    util::{ab_query_func, intersects, AbQueryArgs, Line},
    vector2::Vector2,
};

use nalgebra::Point2;
use parry2d::bounding_volume::Aabb as AABB;
use pi_slotmap::{DefaultKey, SlotMap};
use pi_spatial::{quad_helper::QuadTree, tilemap::TileMap};
use pi_wy_rng::WyRng;
use rand::Rng;
use rand_core::SeedableRng;
pub struct RVOSimulator {
    agents_: SlotMap<DefaultKey, Agent>,
    default_agent: Option<Agent>,
    pub global_time: f32,
    obstacles_: SlotMap<DefaultKey, Obstacle>,
    pub time_step: f32,
    tile_map: TileMap<DefaultKey, i32>,
    obstacles_map: QuadTree<DefaultKey, Vec<Vector2>>,
    id_num: usize,
    rng: WyRng,
}

impl RVOSimulator {
    /// 创建一个默认模拟器
    pub fn default() -> Self {
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
            id_num: 0,
            rng: WyRng::seed_from_u64(1000),
        }
    }

    /// 创建一个新的模拟器，并指定参数
    pub fn new(
        time_step: f32,
        neighbor_dist: f32,
        max_neighbors: usize,
        time_horizon: f32,
        time_horizon_obst: f32,
        radius: f32,
        max_speed: f32,
        velocity: &Vector2,
        rng_seed: u32,
    ) -> Self {
        let max = nalgebra::Vector2::new(100f32, 100f32);
        let min = max / 100f32;
        let mut sim_ = Self {
            agents_: Default::default(),
            default_agent: None,
            global_time: 0.0,
            obstacles_: Default::default(),
            time_step,
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
            id_num: 0,
            rng: WyRng::seed_from_u64(rng_seed as u64),
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

    /// 添加一个代理，使用默认的参数
    pub fn add_agent(&mut self, position: &Vector2, speed: f32) -> f64 {
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
        agent.id_ = self.id_num;
        agent.custom_speed = Some(speed);
        self.id_num += 1;

        let ab: AABB = agent.compute_aabb();
        let id = self.agents_.insert(agent);

        self.tile_map.add(id, ab, 0);

        return unsafe { std::mem::transmute(id) };
    }

    /// 添加一个代理并指定代理的属性
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
        agent.id_ = self.id_num;
        self.id_num += 1;

        let ab: AABB = agent.compute_aabb();
        let id = self.agents_.insert(agent);
        // if let Some(a) = self.agents_.get_mut(id) {
        //     a.id_ = id;
        // }

        self.tile_map.add(id, ab, 0);

        unsafe { std::mem::transmute(id) }
    }

    /// 删除代理。
    pub fn remove_agent(&mut self, id: f64) -> bool {
        let id = unsafe { std::mem::transmute(id) };
        if let Some(_) = self.tile_map.remove(id) {
            if let Some(_) = self.agents_.remove(id) {
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
    pub fn add_obstacle(&mut self, vertices: Vec<Vector2>) -> f64 {
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
            let pos = vertices[i];
            obstacle.point_ = pos;

            let a = if i == vertices.len() - 1 { 0 } else { i + 1 };
            obstacle.unit_dir = Vector2::normalize(&(vertices[a] - pos));

            if vertices.len() == 2 {
                obstacle.is_convex = true;
            } else {
                let a = if i == 0 { vertices.len() - 1 } else { i - 1 };
                let b = if i == vertices.len() - 1 { 0 } else { i + 1 };

                obstacle.is_convex = Vector2::left_of(&vertices[a], &pos, &vertices[b]) >= 0.0;
            }

            let id = self.obstacles_.insert(obstacle);
            if let Some(o) = self.obstacles_.get_mut(id) {
                o.id_ = id
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

        return unsafe { std::mem::transmute(id) };
    }

    /// 删除障碍物
    pub fn remove_obstacle(&mut self, id: f64) -> bool {
        let mut key = unsafe { std::mem::transmute(id) };

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

    /// 模拟帧
    pub fn do_step(&mut self) -> bool {
        self.update_tree();
        let mut obstacle_num = Vec::with_capacity(self.agents_.len());

        let mut res = true;
        for (_id, agents) in &mut self.agents_ {
            if let Some(a) = &agents.goal_position {
                if a.x != agents.position_.x || a.y != agents.position_.y {
                    res = false;
                }
            }
            agents.compute_neighbors();
            obstacle_num.push(agents.compute_obstacle_orca());
        }

        if !res {
            let mut index = 0;
            for (_id, agents) in &mut self.agents_ {
                agents.compute_new_velocity(obstacle_num[index]);
                index += 1;
            }

            //将新的速度应用于所有代理，并初始化状态
            for (_id, agents) in &mut self.agents_ {
                agents.update();
            }

            // 叠加时间
            self.global_time += self.time_step;
        }

        res
    }

    /// 更新代理的AABB
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

    /// 获取代理的邻居代理
    pub fn get_agent_agent_neighbor(&self, agent_no: f64, neighbor_no: usize) -> Option<f64> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.get_agent_agent_neighbor(neighbor_no));
        }
        None
    }

    /// 获取代理的最大邻居数
    pub fn get_agent_max_neighbors(&self, agent_no: f64) -> Option<usize> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.max_neighbors);
        }
        None
    }

    /// 获取代理的最大速度
    pub fn get_agent_max_speed(&self, agent_no: f64) -> Option<f32> {
        // return self.agents_[agent_no].max_speed;
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.max_speed);
        }
        None
    }

    /// 获取代理的邻居距离
    pub fn get_agent_neighbor_dist(&self, agent_no: f64) -> Option<f32> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.neighbor_dist);
        }
        None
    }

    /// 获取代理的代理邻居数量
    pub fn get_agent_num_agent_neighbors(&self, agent_no: f64) -> Option<usize> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.get_agent_num_agent_neighbors());
        }
        None
    }

    /// 获取代理的障碍物邻居数量
    pub fn get_agent_num_obstacle_neighbors(&self, agent_no: f64) -> Option<usize> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.get_agent_num_obstacle_neighbors());
        }
        None
    }

    /// 获取代理的orca线数量
    pub fn get_agent_num_orcalines(&self, agent_no: f64) -> Option<usize> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.get_agent_num_orcalines());
        }
        None
    }

    /// 获取代理的障碍物邻居。
    pub fn get_agent_obstacle_neighbor(&self, agent_no: f64, neighbor_no: f64) -> Option<f64> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.get_agent_obstacle_neighbor(neighbor_no as usize));
        }
        None
    }

    /// 获取代理的位置
    pub fn get_agent_position(&self, agent_no: f64) -> Option<Vector2> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.position_);
        }
        None
    }

    /// 获取代理的首选速度。
    pub fn get_agent_pref_velocity(&self, agent_no: f64) -> Option<Vector2> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.pref_velocity);
        }
        None
    }

    /// 获取代理相的半径。
    pub fn get_agent_radius(&self, agent_no: f64) -> Option<f32> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.radius_);
        }
        None
    }

    /// 获取代理相对于代理的时间视野。
    pub fn get_agent_time_horizon(&self, agent_no: f64) -> Option<f32> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.time_horizon);
        }
        None
    }

    /// 获取代理相对于障碍物的时间视野。
    pub fn get_agent_time_horizon_obst(&self, agent_no: f64) -> Option<f32> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.time_horizon);
        }
        None
    }

    /// 获取代理的速度。
    pub fn get_agent_velocity(&self, agent_no: f64) -> Option<Vector2> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.velocity_);
        }
        None
    }

    /// 获取全局累计模拟时间。
    pub fn get_global_time(&self) -> f32 {
        return self.global_time;
    }

    /// 获取代理的数量。
    pub fn get_num_agents(&self) -> usize {
        return self.agents_.len();
    }

    /// 获取障碍物的数量
    pub fn get_num_obstacle_vertices(&self) -> usize {
        return self.obstacles_.len();
    }

    /// 获取障碍物的第一个顶点。
    pub fn get_obstacle_vertex(&self, vertex_no: f64) -> Option<Vector2> {
        let key = unsafe { std::mem::transmute(vertex_no) };
        if let Some(v) = self.obstacles_.get(key) {
            return Some(v.point_);
        }
        None
    }

    /// 获取障碍物的下一个顶点。
    pub fn get_next_obstacle_vertex_no(&self, vertex_no: f64) -> Option<f64> {
        let key = unsafe { std::mem::transmute(vertex_no) };
        if let Some(v) = self.obstacles_.get(key) {
            if let Some(next_obstacle) = self.obstacles_.get(v.next_obstacle) {
                return Some(unsafe { std::mem::transmute(next_obstacle.id_) });
            }
        }
        None
    }

    /// 获取障碍物的前一个顶点。
    pub fn get_prev_obstacle_vertex_no(&self, vertex_no: f64) -> Option<f64> {
        let key = unsafe { std::mem::transmute(vertex_no) };
        if let Some(v) = self.obstacles_.get(key) {
            if let Some(prev_obstacle) = self.obstacles_.get(v.prev_obstacle) {
                return Some(unsafe { std::mem::transmute(prev_obstacle.id_) });
            }
        }
        None
    }

    /// 获取时间步长。
    pub fn get_time_step(&self) -> f32 {
        return self.time_step;
    }

    /// 设置代理的最大邻居数。
    pub fn set_agent_max_neighbors(&mut self, agent_no: f64, max_neighbors: usize) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.max_neighbors = max_neighbors;
            return true;
        }
        false
    }

    /// 设置代理的最大速度。
    pub fn set_agent_max_speed(&mut self, agent_no: f64, max_speed: f32) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.max_speed = max_speed;
            return true;
        }
        false
    }

    /// 设置代理的邻居距离。
    pub fn set_agent_neighbor_dist(&mut self, agent_no: f64, neighbor_dist: f32) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.neighbor_dist = neighbor_dist;
            return true;
        }
        false
    }

    /// 设置代理的位置。
    pub fn set_agent_position(&mut self, agent_no: f64, position: &Vector2) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.position_ = *position;
            return true;
        }
        false
    }

    /// 设置代理的首选速度。
    pub fn set_agent_pref_velocity(&mut self, agent_no: f64, pref_velocity: &Vector2) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.pref_velocity = *pref_velocity;
            return true;
        }
        false
    }

    /// 设置代理的半径。
    pub fn set_agent_radius(&mut self, agent_no: f64, radius: f32) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.radius_ = radius;
            return true;
        }
        false
    }

    /**
     * @brief     为代理设置目标点。
     * @param[in] agent_no   代理id
     * @param[in] goal       目标点
     * @return 设置成功返回true，失败返回false。
     */
    pub fn set_agent_goal(&mut self, agent_no: f64, goal: Option<Vector2>) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.goal_position = goal.clone();
            if let Some(goal) = goal {
                agent.calc_pref_velocity(goal);
            } else {
                agent.pref_velocity = Vector2::new(0.0, 0.0);
                agent.velocity_ = Vector2::new(0.0, 0.0);
            }
            return true;
        }
        false
    }

    /**
     * @brief     为代理设置自定义速度。
     * @param[in] agent_no   代理id
     * @param[in] speed      速度，应该小于代理的最大速度，否则使用最大速度。
     * @return 设置成功返回true，失败返回false。
     */
    pub fn set_agent_custom_speed(&mut self, agent_no: f64, speed: f32) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.custom_speed = Some(speed);
            return true;
        }
        false
    }

    /// 设置代理的相对于代理时间视野
    pub fn set_agent_time_horizon(&mut self, agent_no: f64, time_horizon: f32) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.time_horizon = time_horizon;
            return true;
        }
        false
    }

    /// 设置代理的相对于障碍物时间视野
    pub fn set_agent_time_horizon_obst(&mut self, agent_no: f64, time_horizon_obst: f32) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.time_horizon_obst = time_horizon_obst;
            return true;
        }
        false
    }

    /// 设置代理的速度
    pub fn set_agent_velocity(&mut self, agent_no: f64, velocity: &Vector2) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.velocity_ = *velocity;
            return true;
        }
        false
    }

    /// 设置每帧模拟的时间步长
    pub fn set_time_step(&mut self, time_step: f32) {
        self.time_step = time_step;
    }

    /// 设置随机种子
    pub fn set_rng_seed(&mut self, seed: u32) {
        self.rng = WyRng::seed_from_u64(seed as u64);
    }

    /**
     * @brief     计算代理邻居。
     * @param[in] ab   当前代理的AABB
     * @return         返回邻居代理的ID集合
     */
    pub fn compute_neighbors_aabb(&mut self, ab: AABB) -> Vec<DefaultKey> {
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
    pub fn compute_obstacle_aabb(&mut self, ab: AABB) -> Vec<(DefaultKey, Vec<Vector2>)> {
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

    /// 计算代理的新位置
    pub fn get_obstacle(&self, obstacle_no: DefaultKey) -> Option<*const Obstacle> {
        if let Some(v) = self.obstacles_.get(obstacle_no) {
            return Some(v);
        }
        None
    }

    /// 获取代理
    pub fn get_agent(&mut self, agent_no: DefaultKey) -> Option<*mut Agent> {
        if let Some(v) = self.agents_.get_mut(agent_no) {
            return Some(v);
        }
        None
    }

    /// 获取代理的orca线
    pub fn get_agent_orcaline(&self, agent_no: f64, line_no: usize) -> Option<Line> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(v) = self.agents_.get(agent_no) {
            return Some(v.get_agent_orcaline(line_no));
        }
        None
    }

    /// 获取代理的目标位置
    pub fn get_agent_goal(&mut self, agent_no: f64) -> Option<Vector2> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get(agent_no) {
            return agent.goal_position;
        }
        None
    }

    /// 获取代理的自定义速度
    pub fn get_agent_custom_speed(&mut self, agent_no: f64) -> Option<f32> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get(agent_no) {
            return agent.custom_speed;
        }
        None
    }

    /// 设置代理的质量; 1.0为默认值, 必须大于0
    pub fn set_agent_quality(&mut self, agent_no: f64, quality: f64) -> bool {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            agent.quality = quality as f32;
            return true;
        }
        false
    }

    /// 获取代理的质量
    pub fn get_agent_quality(&mut self, agent_no: f64) -> Option<f64> {
        let agent_no = unsafe { std::mem::transmute(agent_no) };
        if let Some(agent) = self.agents_.get_mut(agent_no) {
            return Some(agent.quality as f64);
        }
        None
    }
}

impl RVOSimulator {
    pub fn get_rand(&mut self) -> f32 {
        self.rng.gen::<f32>()
    }
}
