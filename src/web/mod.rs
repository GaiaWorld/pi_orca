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

    pub fn do_step(&mut self) {
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
}

// #[wasm_bindgen]
// pub struct Vector2(Vector2Inner);

// impl Vector2 {
//     pub fn default() -> JsValue {
//         JsValue::from(Self(Vector2Inner::default()))
//     }

//     pub fn new(x: f32, y: f32) -> JsValue {
//         JsValue::from(Self(Vector2Inner::new(x, y)))
//     }

//     pub fn x(&self) -> f32 {
//         self.0.x()
//     }

//     pub fn y(&self) -> f32 {
//         self.0.y()
//     }

//     pub fn add(&self, other: JsValue) -> Vector2 {
//         let other: Vector2 = other.into();
//         JsValue::from_serde(Self(self.0.add(&other.0)))
//     }

//     pub fn sub(&self, other: &Vector2) -> Vector2 {
//         Self(self.0.sub(&other.0))
//     }

//     pub fn mul(&self, other: &Vector2) -> f32 {
//         self.0.mul(&other.0)
//     }

//     pub fn mul_number(&self, other: f32) -> Vector2 {
//         Self(self.0.mul_number(other))
//     }

//     pub fn div(&self, other: f32) -> Vector2 {
//         Self(self.0.div(other))
//     }

//     pub fn neg(&self) -> Vector2 {
//         Self(self.0.neg())
//     }

//     pub fn abs(v: &Vector2) -> f32 {
//         Vector2Inner::abs(&v.0)
//     }

//     pub fn abs_sq(v: &Vector2) -> f32 {
//         Vector2Inner::abs_sq(&v.0)
//     }

//     pub fn det(v1: &Vector2, v2: &Vector2) -> f32 {
//         Vector2Inner::det(&v1.0, &v2.0)
//     }

//     pub fn normalize(vector: &Vector2) -> Vector2 {
//         Self(Vector2Inner::normalize(&vector.0))
//     }

//     /**
//      * \brief      Computes the squared distance from a line segment with the
//      *             specified endpoints to a specified point.
//      * \param      a               The first endpoint of the line segment.
//      * \param      b               The second endpoint of the line segment.
//      * \param      c               The point to which the squared distance is to
//      *                             be calculated.
//      * \return     The squared distance from the line segment to the point.
//      */
//     pub fn dist_sq_point_line_segment(a: &Vector2, b: &Vector2, c: &Vector2) -> f32 {}

//     /**
//      * \brief      Computes the signed distance from a line connecting the
//      *             specified points to a specified point.
//      * \param      a               The first point on the line.
//      * \param      b               The second point on the line.
//      * \param      c               The point to which the signed distance is to
//      *                             be calculated.
//      * \return     Positive when the point c lies to the left of the line ab.
//      */
//     pub fn left_of(a: &Vector2, b: &Vector2, c: &Vector2) -> f32 {
//         return Self::det(&(*a - *c), &(*b - *a));
//     }

//     /**
//      * \brief      Computes the square of a float.
//      * \param      a               The float to be squared.
//      * \return     The square of the float.
//      */
//     pub fn sqr(a: f32) -> f32 {
//         return a * a;
//     }
// }
