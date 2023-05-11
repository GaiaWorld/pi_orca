#![cfg(target_arch = "wasm32")]

use crate::rvos_imulator::RVOSimulator as RVOSimulatorInner;
use crate::vector2::Vector2;
use serde_wasm_bindgen::to_value;
use wasm_bindgen::{prelude::wasm_bindgen, JsValue};

#[wasm_bindgen]
pub struct RVOSimulator(RVOSimulatorInner);

#[wasm_bindgen]
impl RVOSimulator {
    pub fn new(
        time_step: f32,
        neighbor_dist: f32,
        max_neighbors: usize,
        time_horizon: f32,
        time_horizon_obst: f32,
        radius: f32,
        max_speed: f32,
        velocity: &[f32],
    ) -> Self {
        let velocity = Vector2::new(velocity[0], velocity[1]);
        Self(RVOSimulatorInner::new(
            time_step,
            neighbor_dist,
            max_neighbors,
            time_horizon,
            time_horizon_obst,
            radius,
            max_speed,
            &velocity,
        ))
    }

    pub fn default() -> Self {
        Self(RVOSimulatorInner::default())
    }

    pub fn add_agent(&mut self, position: &[f32]) -> f64 {
        let pos = Vector2::new(position[0], position[1]);
        self.0.add_agent(&pos)
    }

    pub fn add_agent2(
        &mut self,
        position: &[f32],
        neighbor_dist: f32,
        max_neighbors: usize,
        time_horizon: f32,
        time_horizon_obst: f32,
        radius: f32,
        max_speed: f32,
        velocity: &[f32],
    ) -> f64 {
        let position = Vector2::new(position[0], position[1]);
        let velocity = Vector2::new(velocity[0], velocity[1]);
        self.0.add_agent2(
            &position,
            neighbor_dist,
            max_neighbors,
            time_horizon,
            time_horizon_obst,
            radius,
            max_speed,
            &velocity,
        )
    }

    pub fn remove_agent(&mut self, id: f64) -> bool {
        self.0.remove_agent(id)
    }

    /**
     * @brief 为模拟添加新障碍。
     * @param[in] vertices 逆时针顺序排列的多边形障碍物的顶点列表。
     * @return 障碍物第一个顶点的编号，当顶点数小于2时为返回usize::MAX。
     * @note 要添加“负面”障碍，例如环境周围的边界多边形，顶点应按顺时针顺序列出。
     */
    pub fn add_obstacle(&mut self, vertices: &[f32]) -> f64 {
        let vertices = vertices
            .chunks_exact(2)
            .map(|chunk| Vector2::new(chunk[0], chunk[1]))
            .collect::<Vec<_>>();
        self.0.add_obstacle(vertices)
    }

    pub fn remove_obstacle(&mut self, id: f64) -> bool {
        self.0.remove_obstacle(id)
    }

    pub fn do_step(&mut self)->bool {
        self.0.do_step()
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
        velocity: &[f32],
    ) {
        let velocity = Vector2::new(velocity[0], velocity[1]);
        self.0.set_agent_defaults(
            neighbor_dist,
            max_neighbors,
            time_horizon,
            time_horizon_obst,
            radius,
            max_speed,
            &velocity,
        )
    }

    pub fn get_agent_agent_neighbor(&self, agent_no: f64, neighbor_no: usize) -> Option<f64> {
        self.0.get_agent_agent_neighbor(agent_no, neighbor_no)
    }

    pub fn get_agent_max_neighbors(&self, agent_no: f64) -> Option<usize> {
        self.0.get_agent_max_neighbors(agent_no)
    }

    pub fn get_agent_max_speed(&self, agent_no: f64) -> Option<f32> {
        self.0.get_agent_max_speed(agent_no)
    }

    pub fn get_agent_neighbor_dist(&self, agent_no: f64) -> Option<f32> {
        self.0.get_agent_neighbor_dist(agent_no)
    }

    pub fn get_agent_num_agent_neighbors(&self, agent_no: f64) -> Option<usize> {
        self.0.get_agent_num_agent_neighbors(agent_no)
    }

    pub fn get_agent_num_obstacle_neighbors(&self, agent_no: f64) -> Option<usize> {
        self.0.get_agent_num_obstacle_neighbors(agent_no)
    }

    pub fn get_agent_num_orcalines(&self, agent_no: f64) -> Option<usize> {
        self.0.get_agent_num_orcalines(agent_no)
    }

    pub fn get_agent_obstacle_neighbor(&self, agent_no: f64, neighbor_no: f64) -> Option<f64> {
        self.0.get_agent_obstacle_neighbor(agent_no, neighbor_no)
    }

    pub fn get_agent_position(&self, agent_no: f64) -> JsValue {
        let v = self.0.get_agent_position(agent_no);
        return to_value(&v).unwrap();
    }

    pub fn get_agent_pref_velocity(&self, agent_no: f64) -> JsValue {
        let v = self.0.get_agent_pref_velocity(agent_no);
        return to_value(&v).unwrap();
    }

    pub fn get_agent_radius(&self, agent_no: f64) -> Option<f32> {
        self.0.get_agent_radius(agent_no)
    }

    pub fn get_agent_time_horizon(&self, agent_no: f64) -> Option<f32> {
        self.0.get_agent_time_horizon(agent_no)
    }

    pub fn get_agent_time_horizon_obst(&self, agent_no: f64) -> Option<f32> {
        self.0.get_agent_time_horizon_obst(agent_no)
    }

    pub fn get_agent_velocity(&self, agent_no: f64) -> JsValue {
        let v = self.0.get_agent_velocity(agent_no);
        return to_value(&v).unwrap();
    }

    pub fn get_global_time(&self) -> f32 {
        self.0.get_global_time()
    }

    pub fn get_num_agents(&self) -> usize {
        self.0.get_num_agents()
    }

    pub fn get_num_obstacle_vertices(&self) -> usize {
        self.0.get_num_obstacle_vertices()
    }

    pub fn get_obstacle_vertex(&self, vertex_no: f64) -> JsValue {
        let v = self.0.get_obstacle_vertex(vertex_no);
        to_value(&v).unwrap()
    }

    pub fn get_next_obstacle_vertex_no(&self, vertex_no: f64) -> Option<f64> {
        self.0.get_next_obstacle_vertex_no(vertex_no)
    }

    pub fn get_prev_obstacle_vertex_no(&self, vertex_no: f64) -> Option<f64> {
        self.0.get_prev_obstacle_vertex_no(vertex_no)
    }

    pub fn get_time_step(&self) -> f32 {
        self.0.get_time_step()
    }

    pub fn set_agent_max_neighbors(&mut self, agent_no: f64, max_neighbors: usize) -> bool {
        self.0.set_agent_max_neighbors(agent_no, max_neighbors)
    }

    pub fn set_agent_max_speed(&mut self, agent_no: f64, max_speed: f32) -> bool {
        self.0.set_agent_max_speed(agent_no, max_speed)
    }

    pub fn set_agent_neighbor_dist(&mut self, agent_no: f64, neighbor_dist: f32) -> bool {
        self.0.set_agent_neighbor_dist(agent_no, neighbor_dist)
    }

    pub fn set_agent_position(&mut self, agent_no: f64, position: &[f32]) -> bool {
        let position = Vector2::new(position[0], position[1]);
        self.0.set_agent_position(agent_no, &position)
    }

    pub fn set_agent_pref_velocity(&mut self, agent_no: f64, pref_velocity: &[f32]) -> bool {
        let pref_velocity = Vector2::new(pref_velocity[0], pref_velocity[1]);
        self.0.set_agent_pref_velocity(agent_no, &pref_velocity)
    }

    pub fn set_agent_radius(&mut self, agent_no: f64, radius: f32) -> bool {
        self.0.set_agent_radius(agent_no, radius)
    }

    pub fn set_agent_time_horizon(&mut self, agent_no: f64, time_horizon: f32) -> bool {
        self.0.set_agent_time_horizon(agent_no, time_horizon)
    }

    pub fn set_agent_time_horizon_obst(&mut self, agent_no: f64, time_horizon_obst: f32) -> bool {
        self.0
            .set_agent_time_horizon_obst(agent_no, time_horizon_obst)
    }

    // pub fn set_agent_velocity(&mut self, agent_no: f64, velocity: &[f32]) -> bool {
    //     let velocity = Vector2::new(velocity[0], velocity[1]);
    //     self.0.set_agent_velocity(agent_no, &velocity)
    // }

    pub fn set_time_step(&mut self, time_step: f32) {
        self.0.set_time_step(time_step)
    }

    /**
     * @brief     为代理设置目标点。
     * @param[in] agent_no   代理id
     * @param[in] goal       目标点
     * @return 设置成功返回true，失败返回false。
     */
    pub fn set_agent_goal(&mut self, agent_no: f64, goal: &[f32]) -> bool {
        self.0.set_agent_goal(agent_no, goal)
    }

    /**
     * @brief     为代理设置自定义速度。
     * @param[in] agent_no   代理id
     * @param[in] speed      速度，应该小于代理的最大速度，否则使用最大速度。
     * @return 设置成功返回true，失败返回false。
     */
    pub fn set_agent_custom_speed(&mut self, agent_no: f64, speed: f32) -> bool {
        self.0.set_agent_custom_speed(agent_no, speed)
    }

    pub fn get_agent_goal(&mut self, agent_no: f64) -> JsValue {
        let v = self.0.get_agent_goal(agent_no);
        to_value(&v).unwrap()
    }

    pub fn get_agent_custom_speed(&mut self, agent_no: f64) -> Option<f32> {
        self.0.get_agent_custom_speed(agent_no)
    }
}

