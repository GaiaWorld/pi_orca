// use nalgebra::Point2;
use parry2d::{bounding_volume::Aabb as AABB, na::Point2};

use crate::{
    obstacle::Obstacle,
    rvos_imulator::RVOSimulator,
    util::Line,
    vector2::{Vector2, RVO_EPSILON},
};

#[derive(Clone, Debug)]
pub struct Agent {
    agent_neighbors: Vec<(f32, *mut Agent)>,
    pub max_neighbors: usize,
    pub max_speed: f32,
    pub neighbor_dist: f32,
    pub new_velocity: Vector2,
    obstacle_neighbors: Vec<(f32, *const Obstacle)>,
    orca_lines: Vec<Line>,
    pub position_: Vector2,
    pub pref_velocity: Vector2,
    pub radius_: f32,
    pub sim_: *mut RVOSimulator,
    pub time_horizon: f32,
    pub time_horizon_obst: f32,
    pub velocity_: Vector2,
    pub is_static: bool,
    is_computed: bool,
    pub id_: usize,
    pub goal_position: Option<Vector2>,
    pub custom_speed: Option<f32>,
    pub quality: f32,
}

impl Agent {
    pub fn new(sim_: &mut RVOSimulator) -> Self {
        Self {
            agent_neighbors: vec![],
            max_neighbors: 0,
            max_speed: 0.0,
            neighbor_dist: 0.0,
            new_velocity: Vector2::default(),
            obstacle_neighbors: vec![],
            orca_lines: vec![],
            position_: Vector2::default(),
            pref_velocity: Vector2::default(),
            radius_: 50.0,
            sim_,
            time_horizon: 0.0,
            time_horizon_obst: 0.0,
            velocity_: Vector2::default(),
            id_: 0,
            is_computed: false,
            is_static: false,
            goal_position: None,
            custom_speed: None,
            quality: 1.0,
        }
    }

    /// 计算邻居
    pub fn compute_neighbors(&mut self) {
        let sim = unsafe { &mut *self.sim_ };

        self.obstacle_neighbors.clear();
        let range = Vector2::sqr(self.time_horizon_obst * self.max_speed + self.radius_);
        let aabb = AABB {
            mins: Point2::new(
                self.position_.x - range.sqrt(),
                self.position_.y - range.sqrt(),
            ),
            maxs: Point2::new(
                self.position_.x + range.sqrt(),
                self.position_.y + range.sqrt(),
            ),
        };

        let ids = sim.compute_obstacle_aabb(aabb);

        // 计算障碍物邻居。
        for (begin_id, vertices) in ids {
            let len = vertices.len();
            let mut id = begin_id;
            for _ in 0..len {
                let obstacle = unsafe { &*sim.get_obstacle(id).unwrap() };
                let next_obstacle = unsafe { &*sim.get_obstacle(obstacle.next_obstacle).unwrap() };
                id = obstacle.next_obstacle;
                self.insert_obstacle_neighbor(obstacle, next_obstacle, range);
            }
        }

        self.agent_neighbors.clear();

        // 计算代理邻居。
        if self.max_neighbors > 0 {
            let mut _range_sq = Vector2::sqr(self.neighbor_dist);
            let aabb = AABB {
                mins: Point2::new(self.position_.x - _range_sq, self.position_.y - _range_sq),
                maxs: Point2::new(self.position_.x + _range_sq, self.position_.y + _range_sq),
            };
            let ids = sim.compute_neighbors_aabb(aabb);

            for id in ids {
                let agent = unsafe { &mut *sim.get_agent(id).unwrap() };
                // 当代理邻居和自己都是静态的时候，忽略这个代理；邻居
                if self.is_static && agent.is_static {
                    continue;
                }
                // 如果邻居已经计算过了，忽略这个代理邻居。
                if agent.is_computed {
                    continue;
                }
                self.insert_agent_neighbor(agent, &mut _range_sq);
            }
        }

        self.is_computed = true;
    }

    /// 计算新速度。
    pub fn compute_new_velocity(&mut self, num_obst_lines: usize) {
        // self.compute_pref_velocity();
        let inv_time_horizon = 1.0 / self.time_horizon;

        // 创建agent ORCA 线。
        let len = self.agent_neighbors.len();
        for i in 0..len {
            let other = unsafe { &mut *self.agent_neighbors.get_mut(i).unwrap().1 };
            // 计算自己的orca线。
            self.compute_agent_orca(other, inv_time_horizon);

            let inv_time_horizon2 = 1.0 / other.time_horizon;
            // 计算邻居的orca线。
            other.compute_agent_orca(self, inv_time_horizon2);
        }

        let line_fail = Self::linear_program2(
            &self.orca_lines,
            self.max_speed,
            &self.pref_velocity,
            false,
            &mut self.new_velocity,
        );

        if line_fail < self.orca_lines.len() {
            Self::linear_program3(
                &self.orca_lines,
                num_obst_lines,
                line_fail,
                self.max_speed,
                &mut self.new_velocity,
            );
        }
    }

    /// 插入一个代理邻居到代理的列表。
    pub fn insert_agent_neighbor(&mut self, agent: &mut Agent, range_sq: &mut f32) {
        if self.id_ != agent.id_ {
            let dist_sq = Vector2::abs_sq(&(self.position_ - agent.position_));

            if dist_sq < *range_sq {
                // if self.agent_neighbors.len() < self.max_neighbors {
                self.agent_neighbors.push((dist_sq, agent));
                // }

                let mut i = self.agent_neighbors.len() - 1;

                // 根据距离排序
                while i != 0 && dist_sq < self.agent_neighbors[i - 1].0 {
                    self.agent_neighbors[i] = self.agent_neighbors[i - 1].clone();
                    i -= 1;
                }

                self.agent_neighbors[i] = (dist_sq, agent);

                // if self.agent_neighbors.len() == self.max_neighbors {
                // *range_sq = self.agent_neighbors.last().unwrap().0;
                // }
            }
        }
    }

    /// 插入一个障碍邻居到代理的列表。
    pub fn insert_obstacle_neighbor(
        &mut self,
        obstacle: &Obstacle,
        next_obstacle: &Obstacle,
        range_sq: f32,
    ) {
        let dist_sq = Vector2::dist_sq_point_line_segment(
            &obstacle.point_,
            &next_obstacle.point_,
            &self.position_,
        );

        if dist_sq < range_sq {
            self.obstacle_neighbors.push((dist_sq, obstacle));

            let mut i = self.obstacle_neighbors.len() - 1;

            while i != 0 && dist_sq < self.obstacle_neighbors[i - 1].0 {
                self.obstacle_neighbors[i] = self.obstacle_neighbors[i - 1];
                i -= 1;
            }

            self.obstacle_neighbors[i] = (dist_sq, obstacle);
        }
    }

    /// 跟新代理的位置。
    pub fn update(&mut self) {
        self.velocity_ = self.new_velocity;
        // 计算下一个点的位置。
        let next_position = self.position_ + self.velocity_ * (unsafe { &*self.sim_ }).time_step;
        // 判断是否到途经目标点。
        if let Some(goal) = &self.goal_position {
            // 如过不是从目标点推开的，那么就需要判断是否到达目标点
            if goal.x != self.position_.x || goal.y != self.position_.y {
                let min_x = self.position_.x.min(next_position.x);
                let min_y = self.position_.y.min(next_position.y);

                let max_x = self.position_.x.max(next_position.x);
                let max_y = self.position_.y.max(next_position.y);
                if intersects(
                    &Vector2::new(min_x, min_y),
                    &Vector2::new(max_x, max_y),
                    goal,
                ) {
                    self.position_ = *goal;
                    self.pref_velocity = Vector2::default();
                    self.velocity_ = Vector2::default();
                    self.is_computed = false;
                    return;
                }
            }
            self.position_ = next_position;
            // 如果当前速度不等于期望速度，则重新计算期望速度
            if self.velocity_.x != self.pref_velocity.x || self.velocity_.y != self.pref_velocity.y
            {
                self.calc_pref_velocity(*goal);
                // 随机一下速度
                let x = (unsafe { &mut *self.sim_ }).get_rand() * self.pref_velocity.x * 0.02;
                let y = (unsafe { &mut *self.sim_ }).get_rand() * self.pref_velocity.y * 0.02;
                let v = Vector2::new(x, y);
                // 使用部分上一帧的速度，部分当前帧的期望速度和随机值
                self.velocity_ = v + self.velocity_ * 0.98;
            }
        } else {
            self.position_ = next_position;
        }
        self.is_computed = false;
    }

    /// 重新计算代理的预期速度
    pub fn calc_pref_velocity(&mut self, goal: Vector2) {
        let speed = if self.custom_speed.is_some() && self.max_speed > self.custom_speed.unwrap() {
            self.custom_speed.unwrap()
        } else {
            self.max_speed
        };
        let dist_pos = goal - &self.position_;
        // 根据最大速度或者自定义速度计算pref_velocity
        // 当距离过近时，pref_velocity为dist_pos
        if Vector2::abs_sq(&dist_pos) > speed * speed {
            self.pref_velocity = Vector2::normalize(&dist_pos).mul_number(speed);
        } else {
            self.pref_velocity = dist_pos;
        }
    }

    pub fn linear_program1(
        lines: &Vec<Line>,
        line_no: usize,
        radius: f32,
        opt_velocity: &Vector2,
        direction_opt: bool,
        result: &mut Vector2,
    ) -> bool {
        let dot_product = lines[line_no].point * lines[line_no].direction;
        let discriminant = Vector2::sqr(dot_product) + Vector2::sqr(radius)
            - Vector2::abs_sq(&lines[line_no].point);

        if discriminant < 0.0 {
            /* 最大速度 圆圈 完全无效线 */
            return false;
        }

        let sqrt_discriminant = discriminant.sqrt();
        let mut t_left = -dot_product - sqrt_discriminant;
        let mut t_right = -dot_product + sqrt_discriminant;

        for i in 0..line_no {
            let denominator = Vector2::det(&lines[line_no].direction, &lines[i].direction);
            let numerator = Vector2::det(
                &lines[i].direction,
                &(lines[line_no].point - lines[i].point),
            );

            if denominator.abs() <= RVO_EPSILON {
                /* 线 lineNo 和 i 是（几乎）平行的。 */
                if numerator < 0.0 {
                    return false;
                } else {
                    continue;
                }
            }

            let t = numerator / denominator;

            if denominator >= 0.0 {
                /* 第 i 行边界线 在lineNo 右边。 */
                t_right = t_right.min(t);
            } else {
                /* 第 i 行边界线 在lineNo 左边。 */
                t_left = t.max(t_left);
            }

            if t_left > t_right {
                return false;
            }
        }

        if direction_opt {
            /* 优化方向。 */
            if lines[line_no].direction * opt_velocity > 0.0 {
                /* 采取左端。 */
                *result = lines[line_no].point + lines[line_no].direction * t_right;
            } else {
                /* 采取右端。*/
                *result = lines[line_no].point + lines[line_no].direction * t_left;
            }
        } else {
            /* 优化最近点。 */
            let t: f32 = lines[line_no].direction * (*opt_velocity - lines[line_no].point);
            if t < t_left {
                *result = lines[line_no].point + lines[line_no].direction * t_left;
            } else if t > t_right {
                *result = lines[line_no].point + lines[line_no].direction * t_right;
            } else {
                *result = lines[line_no].point + lines[line_no].direction * t;
            }
        }

        return true;
    }

    pub fn linear_program2(
        lines: &Vec<Line>,
        radius: f32,
        opt_velocity: &Vector2,
        direction_opt: bool,
        result: &mut Vector2,
    ) -> usize {
        if direction_opt {
            /*
             * 请注意，在这种情况下，优化速度是单位长度的。
             */
            *result = *opt_velocity * radius;
        } else if Vector2::abs_sq(opt_velocity) > Vector2::sqr(radius) {
            /*优化最近点和外圆。 */
            *result = Vector2::normalize(opt_velocity) * radius;
        } else {
            /* 优化最近点和内圆。*/
            *result = *opt_velocity;
        }

        // let temp = *result;
        for i in 0..lines.len() {
            // let r = Vector2::det(&lines[i].direction, &(lines[i].point - *result));
            if Vector2::det(&lines[i].direction, &(lines[i].point - *result)) > 0.0 {
                /*结果不满足约束 i。 计算新的最优结果。 */
                let temp_result = *result;
                if !Self::linear_program1(lines, i, radius, opt_velocity, direction_opt, result) {
                    *result = temp_result;
                    return i;
                }
            }
        }

        return lines.len();
    }

    pub fn linear_program3(
        lines: &Vec<Line>,
        num_obst_lines: usize,
        begin_line: usize,
        radius: f32,
        result: &mut Vector2,
    ) {
        let mut distance = 0.0;

        for i in begin_line..lines.len() {
            if Vector2::det(&lines[i].direction, &(lines[i].point - *result)) > distance {
                /* 结果不满足第 i 行的约束。*/
                let mut proj_lines = vec![];

                lines[0..num_obst_lines].iter().for_each(|l| {
                    proj_lines.push(l.clone());
                });

                for j in num_obst_lines..i {
                    let mut line = Line::default();

                    let determinant = Vector2::det(&lines[i].direction, &lines[j].direction);

                    if determinant.abs() <= RVO_EPSILON {
                        /* 线 i 和线 j 平行。 */
                        if lines[i].direction * lines[j].direction > 0.0 {
                            /* i 和 j 线指向同一方向。*/
                            continue;
                        } else {
                            /* 线 i 和线 j 指向相反的方向。*/
                            line.point = (lines[i].point + lines[j].point) * 0.5;
                        }
                    } else {
                        line.point = lines[i].point
                            + lines[i].direction
                                * (Vector2::det(
                                    &lines[j].direction,
                                    &(lines[i].point - lines[j].point),
                                ) / determinant);
                    }

                    line.direction = Vector2::normalize(&(lines[j].direction - lines[i].direction));
                    proj_lines.push(line);
                }

                let temp_result = *result;

                if Self::linear_program2(
                    &proj_lines,
                    radius,
                    &Vector2::new(-lines[i].direction.y, lines[i].direction.x),
                    true,
                    result,
                ) < proj_lines.len()
                {
                    // 这在原则上不应该发生。
                    // 根据定义，结果已经在该线性程序的可行域中。 如果失败，则为小浮点误差，保留当前结果。
                    *result = temp_result;
                }

                distance = Vector2::det(&lines[i].direction, &(lines[i].point - *result));
            }
        }
    }

    pub fn get_agent_num_agent_neighbors(&self) -> usize {
        self.agent_neighbors.len()
    }

    pub fn get_agent_num_obstacle_neighbors(&self) -> usize {
        self.obstacle_neighbors.len()
    }

    pub fn get_agent_num_orcalines(&self) -> usize {
        return self.orca_lines.len();
    }

    pub fn get_agent_obstacle_neighbor(&self, neighbor_no: usize) -> f64 {
        return unsafe { std::mem::transmute((&*self.obstacle_neighbors[neighbor_no].1).id_) };
    }

    pub fn get_agent_orcaline(&self, line_no: usize) -> Line {
        return self.orca_lines[line_no];
    }

    pub fn get_agent_agent_neighbor(&self, neighbor_no: usize) -> f64 {
        return unsafe { (&*self.agent_neighbors[neighbor_no].1).id_ as f64 };
    }
}

impl Agent {
    /// 计算代理的AABB包围盒
    pub fn compute_aabb(&self) -> AABB {
        AABB {
            mins: Point2::new(
                self.position_.x - self.radius_,
                self.position_.y - self.radius_,
            ),
            maxs: Point2::new(
                self.position_.x + self.radius_,
                self.position_.y + self.radius_,
            ),
        }
    }

    /// 计算代理的orca线
    pub fn compute_agent_orca(&mut self, other: &Agent, inv_time_horizon: f32) {
        let total_quality = self.quality + other.quality;
        // 相对位置
        let relative_position = other.position_ - self.position_;
        // 相对速度
        let relative_velocity = self.velocity_ - other.velocity_;
        let dist_sq = Vector2::abs_sq(&relative_position);
        // 半径和
        let combined_radius = self.radius_ + other.radius_;
        let combined_radius_sq = Vector2::sqr(combined_radius);

        let mut line = Line::default();
        let mut _u: Vector2 = Vector2::default();

        if dist_sq > combined_radius_sq {
            /* 没有碰撞。 */
            // 这里主要要参考那张一个大圆一个小园，在一个锥体内的那张图
            // 当前的相对速度相对于发生碰摔的临界速度的余量
            // 可以理解为在速度域中，撞击速度指向当前速康的向量
            // 在八何表示中，就是小圆中心指向当前速度点的向量
            /* 从截止中心到相对速度的矢量。*/
            let w: Vector2 = relative_velocity - relative_position * inv_time_horizon;
            // 此向量的平方
            let w_length_sq = Vector2::abs_sq(&w); // 1

            // 此向量到碰撞方向的投影
            //在几何表示中，relativeposition即为大圆中心，此处可以理解为此向量到园锥中心向量的投影
            let dot_product1 = w * relative_position; // 1.1

            // 当前速度相对于碰撞速度的余量
            // 到碰撞方向的投影小于0，说明目前速度在碰撞方向的速度还没到碰撞需要的速度
            // 所以其在速度空间中的仅晋应该位干障碍区域的前方
            // 后面一个条件，可以同时开根，即当dotProduct1>combinedRadius *wLength时
            // 此时消掉两边的wlength，得到的就是大园中心向量到的投影 >conbinedRadius，注意比较符号两侧都是取绝对值得
            // 这个条件放到几何表示中比较明显，其实就是在比较w向量和两条腿的重线的角度
            // 因为我们此处只考虑障碍区域前侧，所以w与relativePosition一定是钝角
            // 当此角度确定为钝角时，它的角度越大，投影的绝对值就越大，直到180度时cos为-1
            // 所以这个条件相当于是拿到了我们参考图中前侧小园的弧线部分，即障碍区城的前侧边缘
            if dot_product1 < 0.0 && Vector2::sqr(dot_product1) > combined_radius_sq * w_length_sq {
                /* 最近的点在圆上 */
                let w_length: f32 = w_length_sq.sqrt();
                let unit_w = w / w_length;
                // 方向为w顺时针转90°，相当于是当前速度相对于小圆的切线
                line.direction = Vector2::new(unit_w.y, -unit_w.x);
                // 此处combined_radius * inv_time_horizon为小圆半径
                // 减去wLength就是把w放在小圆圆心时，w终点到小圆表面的距高
                _u = unit_w * (combined_radius * inv_time_horizon - w_length);
                // _u = unit_w * combined_radius * inv_time_horizon - w;
            } else {
                /* 最近的点在腿上 */
                // 勾股定埋，计算出从原点开始的两京腿的长底
                let leg = (dist_sq - combined_radius_sq).sqrt();

                // 此处是第一列为relative_position第二列为的行列式
                // 行列式司以理解为有高 面积（或体积）因此可以近过行列式正负来判断行列式第一列向量到第二列向量的旋转方向（180度以内)
                // 大于0时是逆时针旋转，小于0是顺时针
                if Vector2::det(&relative_position, &w) > 0.0 {
                    /* 最近的点在左腿上 */
                    // 这里可以把dist_sq除进来，就可以理解为是一个向量旋转的操作
                    line.direction = Vector2::new(
                        relative_position.x * leg - relative_position.y * combined_radius,
                        relative_position.x * combined_radius + relative_position.y * leg,
                    ) / dist_sq;
                } else {
                    /* 最近的点在右腿上 */
                    // 这里可以把dist_sq除进来，就可以理解为是一个向量旋转的操作
                    line.direction = -Vector2::new(
                        relative_position.x * leg + relative_position.y * combined_radius,
                        -relative_position.x * combined_radius + relative_position.y * leg,
                    ) / dist_sq;
                }
                let dot_product2 = relative_velocity * line.direction;
                // 此址u应该是当前速度到腿的年线
                _u = line.direction * dot_product2 - relative_velocity;
            }
        } else {
            /* 碰撞*/
            let inv_time_step = 1.0 / (unsafe { &*self.sim_ }).time_step;

            /* 从截止中心到相对速度的矢量。 */
            let w = relative_velocity - relative_position * inv_time_step;

            let w_length = Vector2::abs(&w);
            let unit_w = w / w_length;

            line.direction = Vector2::new(unit_w.y, -unit_w.x);
            _u = unit_w * (combined_radius * inv_time_step - w_length);
        }
        // line.point = self.velocity_ + _u * 0.5;
        // 质量越大的，其避让的权重越小
        line.point = self.velocity_ + _u * (1.0 - self.quality / total_quality);

        self.orca_lines.push(line);
    }

    /// 计算障碍物的 ORCA 线
    pub fn compute_obstacle_orca(&mut self) -> usize {
        self.orca_lines.clear();

        let sim = unsafe { &*self.sim_ };

        let inv_time_horizon_obst = 1.0 / self.time_horizon_obst;

        // 计算障碍物的 ORCA 线
        for i in 0..self.obstacle_neighbors.len() {
            let mut obstacle1 = unsafe { &*self.obstacle_neighbors.get_mut(i).unwrap().1 };
            let mut obstacle2 = unsafe { &*sim.get_obstacle(obstacle1.next_obstacle).unwrap() };

            let relative_position1 = obstacle1.point_ - self.position_;
            let relative_position2 = obstacle2.point_ - self.position_;

            // 检查障碍物的速度障碍物是否已被先前构建的障碍物 ORCA 线处理。
            let mut already_covered = false;

            for j in 0..self.orca_lines.len() {
                let a = Vector2::det(
                    &(relative_position1 * inv_time_horizon_obst - self.orca_lines[j].point),
                    &self.orca_lines[j].direction,
                ) - inv_time_horizon_obst * self.radius_;
                let b = Vector2::det(
                    &(relative_position2 * inv_time_horizon_obst - self.orca_lines[j].point),
                    &self.orca_lines[j].direction,
                ) - inv_time_horizon_obst * self.radius_;
                if a >= -RVO_EPSILON && b >= -RVO_EPSILON {
                    already_covered = true;
                    break;
                }
            }

            if already_covered {
                continue;
            }

            // 尚未涵盖, 检查碰撞。
            let dist_sq1 = Vector2::abs_sq(&relative_position1);
            let dist_sq2 = Vector2::abs_sq(&relative_position2);

            let radius_sq = Vector2::sqr(self.radius_);

            let obstacle_vector = obstacle2.point_ - obstacle1.point_;
            let s = ((-relative_position1) * obstacle_vector) / Vector2::abs_sq(&obstacle_vector);
            let dist_sq_line = Vector2::abs_sq(&((-relative_position1) - obstacle_vector * s));

            let mut line = Line::default();

            if s < 0.0 && dist_sq1 <= radius_sq {
                // 与左顶点碰撞, 如果非凸则忽略。
                if obstacle1.is_convex {
                    line.point = Vector2::new(0.0, 0.0);
                    line.direction = Vector2::normalize(&Vector2::new(
                        -relative_position1.y,
                        relative_position1.x,
                    ));
                    self.orca_lines.push(line);
                }
                continue;
            } else if s > 1.0 && dist_sq2 <= radius_sq {
                // 与右顶点碰撞。 忽略非凸面或是否会被相邻障碍物处理
                if obstacle2.is_convex
                    && Vector2::det(&relative_position2, &obstacle2.unit_dir) >= 0.0
                {
                    line.point = Vector2::new(0.0, 0.0);
                    line.direction = Vector2::normalize(&Vector2::new(
                        -relative_position2.y,
                        relative_position2.x,
                    ));
                    self.orca_lines.push(line);
                }
                continue;
            } else if s >= 0.0 && s < 1.0 && dist_sq_line <= radius_sq {
                // 与障碍物相撞。
                line.point = Vector2::new(0.0, 0.0);
                line.direction = -obstacle1.unit_dir;
                self.orca_lines.push(line);

                continue;
            }

            // 无碰撞, 计算leg
            // 斜视时，两条leg都可以来自一个顶点。 非凸顶点时leg延伸截止线。
            let mut _left_leg_direction = Vector2::default();
            let mut _right_leg_direction = Vector2::default();

            if s < 0.0 && dist_sq_line <= radius_sq {
                // 倾斜地观察障碍物，以便左顶点定义速度障碍物。
                if !obstacle1.is_convex {
                    /* 忽略障碍物。 */
                    continue;
                }
                obstacle2 = obstacle1;

                let leg1 = (dist_sq1 - radius_sq).sqrt();
                _left_leg_direction = Vector2::new(
                    relative_position1.x * leg1 - relative_position1.y * self.radius_,
                    relative_position1.x * self.radius_ + relative_position1.y * leg1,
                ) / dist_sq1;
                _right_leg_direction = Vector2::new(
                    relative_position1.x * leg1 + relative_position1.y * self.radius_,
                    -relative_position1.x * self.radius_ + relative_position1.y * leg1,
                ) / dist_sq1;
            } else if s > 1.0 && dist_sq_line <= radius_sq {
                // 倾斜地观察障碍物，以便右顶点定义速度障碍物。
                if !obstacle2.is_convex {
                    /* 忽略障碍物。 */
                    continue;
                }

                obstacle1 = obstacle2;

                let leg2 = (dist_sq2 - radius_sq).sqrt();
                // println!("_left_leg_direction2");
                _left_leg_direction = Vector2::new(
                    relative_position2.x * leg2 - relative_position2.y * self.radius_,
                    relative_position2.x * self.radius_ + relative_position2.y * leg2,
                ) / dist_sq2;
                _right_leg_direction = Vector2::new(
                    relative_position2.x * leg2 + relative_position2.y * self.radius_,
                    -relative_position2.x * self.radius_ + relative_position2.y * leg2,
                ) / dist_sq2;
            } else {
                /* 平时的情况。 */
                if obstacle1.is_convex {
                    let leg1 = (dist_sq1 - radius_sq).sqrt();
                    // println!("_left_leg_direction3");
                    _left_leg_direction = Vector2::new(
                        relative_position1.x * leg1 - relative_position1.y * self.radius_,
                        relative_position1.x * self.radius_ + relative_position1.y * leg1,
                    ) / dist_sq1;
                } else {
                    /* 左顶点非凸； 左leg延伸至截止线。 */
                    // println!("_left_leg_direction4");
                    _left_leg_direction = -obstacle1.unit_dir;
                }

                if obstacle2.is_convex {
                    let leg2 = (dist_sq2 - radius_sq).sqrt();
                    _right_leg_direction = Vector2::new(
                        relative_position2.x * leg2 + relative_position2.y * self.radius_,
                        -relative_position2.x * self.radius_ + relative_position2.y * leg2,
                    ) / dist_sq2;
                } else {
                    /* 右顶点非凸； 右leg延伸截止线。 */
                    _right_leg_direction = obstacle1.unit_dir;
                }
            }

            // 当凸顶点时，leg永远不能指向相邻边，取而代之的是相邻边的截止线。
            // 如果速度投射在“外”leg上，则不添加任何约束。
            let left_neighbor = unsafe { &*sim.get_obstacle(obstacle1.prev_obstacle).unwrap() };

            let mut is_left_leg_foreign = false;
            let mut is_right_leg_foreign = false;

            if obstacle1.is_convex
                && Vector2::det(&_left_leg_direction, &-left_neighbor.unit_dir) >= 0.0
            {
                // 左leg指向障碍物。
                _left_leg_direction = -left_neighbor.unit_dir;
                is_left_leg_foreign = true;
            }

            if obstacle2.is_convex
                && Vector2::det(&_right_leg_direction, &obstacle2.unit_dir) <= 0.0
            {
                // 右leg指向障碍物。
                _right_leg_direction = obstacle2.unit_dir;
                is_right_leg_foreign = true;
            }

            // 计算截止中心,
            let left_cutoff = (obstacle1.point_ - self.position_) * inv_time_horizon_obst;
            let right_cutoff = (obstacle2.point_ - self.position_) * inv_time_horizon_obst;
            let cutoff_vec = right_cutoff - left_cutoff;

            // 将当前速度投射到速度障碍物上。
            // 检查当前速度是否投影在截止圆上。
            let t: f32 = if obstacle1.id_ == obstacle2.id_ {
                0.5
            } else {
                ((self.velocity_ - left_cutoff) * cutoff_vec) / Vector2::abs_sq(&cutoff_vec)
            };

            let t_left = (self.velocity_ - left_cutoff) * _left_leg_direction;
            let t_right = (self.velocity_ - right_cutoff) * _right_leg_direction;

            if (t < 0.0 && t_left < 0.0)
                || (obstacle1.id_ == obstacle2.id_ && t_left < 0.0 && t_right < 0.0)
            {
                /* 投影在左截止圆上 */
                let unit_w = Vector2::normalize(&(self.velocity_ - left_cutoff));

                line.direction = Vector2::new(unit_w.y, -unit_w.x);
                line.point = left_cutoff + unit_w * (self.radius_ * inv_time_horizon_obst);
                self.orca_lines.push(line);
                continue;
            } else if t > 1.0 && t_right < 0.0 {
                /* 投影在右截止圆上。*/
                let unit_w = Vector2::normalize(&(self.velocity_ - right_cutoff));

                line.direction = Vector2::new(unit_w.y, -unit_w.x);
                line.point = right_cutoff + unit_w * self.radius_ * inv_time_horizon_obst;
                self.orca_lines.push(line);

                continue;
            }

            // 投影在左leg、右leg或截止线上，以最接近速度的为准。
            let dist_sq_cutoff = if t < 0.0 || t > 1.0 || obstacle1.id_ == obstacle2.id_ {
                f32::MAX
            } else {
                Vector2::abs_sq(&(self.velocity_ - (left_cutoff + cutoff_vec * t)))
            };
            let dist_sq_left = if t_left < 0.0 {
                f32::MAX
            } else {
                Vector2::abs_sq(&(self.velocity_ - (left_cutoff + _left_leg_direction * t_left)))
            };
            let dist_sq_right = if t_right < 0.0 {
                f32::MAX
            } else {
                Vector2::abs_sq(&(self.velocity_ - (right_cutoff + _right_leg_direction * t_right)))
            };

            if dist_sq_cutoff <= dist_sq_left && dist_sq_cutoff <= dist_sq_right {
                /* 对象在截止线上。 */
                line.direction = -obstacle1.unit_dir;
                line.point = left_cutoff
                    + Vector2::new(-line.direction.y, line.direction.x)
                        * self.radius_
                        * inv_time_horizon_obst;
                self.orca_lines.push(line);
                continue;
            } else if dist_sq_left <= dist_sq_right {
                /* 对象在左leg上。 */
                if is_left_leg_foreign {
                    continue;
                }

                line.direction = _left_leg_direction;
                line.point = left_cutoff
                    + Vector2::new(-line.direction.y, line.direction.x)
                        * self.radius_
                        * inv_time_horizon_obst;
                self.orca_lines.push(line);
                continue;
            } else {
                /* 对象在右leg上。 */
                if is_right_leg_foreign {
                    continue;
                }

                line.direction = -_right_leg_direction;
                line.point = right_cutoff
                    + Vector2::new(-line.direction.y, line.direction.x)
                        * self.radius_
                        * inv_time_horizon_obst;
                self.orca_lines.push(line);
                continue;
            }
        }

        self.orca_lines.len()
    }
}

/// 判断直线p1p2与圆c是否相交，相交返回true，否则返回false
pub fn judge(p1: Vector2, p2: Vector2, cricle_pos: Vector2, cricle_radius: f32) -> bool {
    let flag1 = (p1.x - cricle_pos.x) * (p1.x - cricle_pos.x)
        + (p1.y - cricle_pos.y) * (p1.y - cricle_pos.y)
        <= cricle_radius * cricle_radius;
    let flag2 = (p2.x - cricle_pos.x) * (p2.x - cricle_pos.x)
        + (p2.y - cricle_pos.y) * (p2.y - cricle_pos.y)
        <= cricle_radius * cricle_radius;

    if flag1 && flag2
    //情况一、两点都在圆内 :一定不相交
    {
        return true;
    } else if flag1 || flag2
    //情况二、一个点在圆内，一个点在圆外：一定相交
    {
        return true;
    } else
    //情况三、两个点都在圆外
    {
        //  A,B,C,dist1,dist2,angle1,angle2;
        //将直线p1p2化为一般式：Ax+By+C=0的形式。先化为两点式，然后由两点式得出一般式
        let a = p1.y - p2.y;
        let b = p2.x - p1.x;
        let c = p1.x * p2.y - p2.x * p1.y;
        //使用距离公式判断圆心到直线ax+by+cricle_pos=0的距离是否大于半径
        let mut dist1 = a * cricle_pos.x + b * cricle_pos.y + c;
        dist1 *= dist1;
        let dist2 = (a * a + b * b) * cricle_radius * cricle_radius;
        if dist1 > dist2
        //圆心到直线p1p2的距离大于半径，不相交
        {
            return false;
        }
        let angle1 = (cricle_pos.x - p1.x) * (p2.x - p1.x) + (cricle_pos.y - p1.y) * (p2.y - p1.y);
        let angle2 = (cricle_pos.x - p2.x) * (p1.x - p2.x) + (cricle_pos.y - p2.y) * (p1.y - p2.y);
        if angle1 > 0. && angle2 > 0.
        //余弦为正，则是锐角，一定相交
        {
            return true;
        } else {
            return false;
        }
    }
}

/// 判断点是否在aabb内
pub fn intersects(ab_mins: &Vector2, ab_maxs: &Vector2, b: &Vector2) -> bool {
    ab_mins.x <= b.x && ab_maxs.x >= b.x && ab_mins.y <= b.y && ab_maxs.y >= b.y
}
