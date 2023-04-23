use nalgebra::Point2;
use ncollide2d::bounding_volume::AABB;

use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    obstacle::Obstacle,
    rvos_imulator::{RVOSimulator, ID},
    vector2::{Vector2, RVO_EPSILON},
};

#[wasm_bindgen]
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
#[derive(Clone)]
pub struct Agent {
    agent_neighbors: Vec<(f32, *mut Agent)>,
    pub max_neighbors: usize,
    pub max_speed: f32,
    pub neighbor_dist: f32,
    pub new_velocity: Vector2,
    obstacle_neighbors: Vec<(f32, Obstacle)>,
    orca_lines: Vec<Line>,
    pub position_: Vector2,
    pub pref_velocity: Vector2,
    pub radius_: f32,
    pub sim_: *mut RVOSimulator,
    pub time_horizon: f32,
    pub time_horizon_obst: f32,
    pub velocity_: Vector2,

    pub id_: ID,
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
            id_: ID(0),
        }
    }

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
        // if self.id_.0 == 0 {
        //     println!("ids: {:?}", ids.len());
        // }

        for (begin_id, vertices) in ids {
            let mut intersect = false;
            let len = vertices.len();
            // println!("aabb intersects");
            // println!("len: {}, begin_id: {}", len, begin_id);
            for id in begin_id..begin_id + len {
                let obstacles1 = sim.get_obstacles(id);

                let next_index = if id == begin_id + len - 1 {
                    begin_id
                } else {
                    id + 1
                };
                let obstacles2 = sim.get_obstacles(next_index);
                // println!("obstacles1.point_: {:?}", obstacles1.point_);
                // println!("obstacles2.point_: {:?}", obstacles2.point_);
                // println!("self.position_: {:?}", self.position_);
                // println!("_range_sq: {:?}", range);
                // if judge(
                //     obstacles1.point_,
                //     obstacles2.point_,
                //     self.position_,
                //     range.sqrt(),
                // ) {
                //     // println!("line intersects!! id1 {}", id);
                //     // println!("line intersects!! agent->id {:?}", self.id_);
                //     // println!("line intersects!! _range_sq {}", _range_sq);
                //     // println!("line intersects!! agent.pos {:?}", self.position_);
                //     println!("=======================================");
                //     intersect = true;
                //     break;
                // }
            }
            
            // if intersect {
            for id in begin_id..begin_id + len {
                
                let obstacle = sim.get_obstacles(id);
                // if self.id_.0 == 0 {
                //     println!("obstacle{}: {:?}", id, obstacle);
                // }

                self.insert_obstacle_neighbor(obstacle, range);
            }
            // if self.id_.0 == 0  {
            //     println!("self.obstacle_neighbors: {:?}", self.obstacle_neighbors)
            // }
            // }
        }

        self.agent_neighbors.clear();

        if self.max_neighbors > 0 {
            let mut _range_sq = Vector2::sqr(self.neighbor_dist);
            // sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
            // println!("_range_sq: {}", _range_sq);
            let aabb = AABB {
                mins: Point2::new(self.position_.x - _range_sq, self.position_.y - _range_sq),
                maxs: Point2::new(self.position_.x + _range_sq, self.position_.y + _range_sq),
            };
            let ids = sim.compute_neighbors_aabb(aabb);

            for ids in ids {
                self.insert_agent_neighbor(unsafe { &mut *sim.get_agents(ids) }, &mut _range_sq);
            }
        }
    }

    pub fn compute_new_velocity(&mut self) {
        self.orca_lines.clear();
        let inv_time_horizon_obst = 1.0 / self.time_horizon_obst;
        // if self.id_.0 == 0 {
        //     println!("self.obstacle_neighbors.len(): {:?}", self.obstacle_neighbors.len());
        // }
        for i in 0..self.obstacle_neighbors.len() {
            let mut obstacle1 =  self.obstacle_neighbors.get_mut(i).unwrap().1.clone();
            
            let mut obstacle2 = unsafe { &*obstacle1.next_obstacle }.clone();
            // if self.id_.0 ==0  {
            //     println!("====obstacle1: {:?}", obstacle1);
            //     println!("==== obstacle2: {:?}", obstacle2);
            // }
            let relative_position1 = obstacle1.point_ - self.position_;
            let relative_position2 = obstacle2.point_ - self.position_;

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
                // println!("4444444444444");
                continue;
            }

            let dist_sq1 = Vector2::abs_sq(&relative_position1);
            let dist_sq2 = Vector2::abs_sq(&relative_position2);

            let radius_sq = Vector2::sqr(self.radius_);

            let obstacle_vector = obstacle2.point_ - obstacle1.point_;
            let s = ((-relative_position1) * obstacle_vector) / Vector2::abs_sq(&obstacle_vector);
            let dist_sq_line = Vector2::abs_sq(&((-relative_position1) - obstacle_vector * s));

            let mut line = Line::default();

            if s < 0.0 && dist_sq1 <= radius_sq {
                /* Collision with left vertex. Ignore if non-convex. */
                if obstacle1.is_convex {
                    line.point = Vector2::new(0.0, 0.0);
                    line.direction = Vector2::normalize(&Vector2::new(
                        -relative_position1.y(),
                        relative_position1.x(),
                    ));
                    // if self.id_.0 == 0 {
                    //     println!("line0000000000");
                    // }
                    self.orca_lines.push(line);
                }
                // println!("3333333333333333");
                continue;
            } else if s > 1.0 && dist_sq2 <= radius_sq {
                /* Collision with right vertex. Ignore if non-convex
                 * or if it will be taken care of by neighoring obstace */
                if obstacle2.is_convex
                    && Vector2::det(&relative_position2, &obstacle2.unit_dir) >= 0.0
                {
                    line.point = Vector2::new(0.0, 0.0);
                    line.direction = Vector2::normalize(&Vector2::new(
                        -relative_position2.y(),
                        relative_position2.x(),
                    ));
                    if self.id_.0 == 0 {
                        println!("line11111111111");
                    }

                    self.orca_lines.push(line);
                }
                // println!("22222222222222");
                continue;
            } else if s >= 0.0 && s < 1.0 && dist_sq_line <= radius_sq {
                /* Collision with obstacle segment. */
                line.point = Vector2::new(0.0, 0.0);
                line.direction = -obstacle1.unit_dir;
                if self.id_.0 == 0 {
                    println!("line222222222222");
                }

                self.orca_lines.push(line);

                continue;
            }

            let mut _left_leg_direction = Vector2::default();
            let mut _right_leg_direction = Vector2::default();

            if s < 0.0 && dist_sq_line <= radius_sq {
                /*
                 * Obstacle viewed obliquely so that left vertex
                 * defines velocity obstacle.
                 */
                if !obstacle1.is_convex {
                    /* Ignore obstacle. */
                    continue;
                }
                // println!("00000000000");
                obstacle2 = obstacle1;

                let leg1 = (dist_sq1 - radius_sq).sqrt();
                println!("_left_leg_direction1");
                _left_leg_direction = Vector2::new(
                    relative_position1.x() * leg1 - relative_position1.y() * self.radius_,
                    relative_position1.x() * self.radius_ + relative_position1.y() * leg1,
                ) / dist_sq1;
                _right_leg_direction = Vector2::new(
                    relative_position1.x() * leg1 + relative_position1.y() * self.radius_,
                    -relative_position1.x() * self.radius_ + relative_position1.y() * leg1,
                ) / dist_sq1;
            } else if s > 1.0 && dist_sq_line <= radius_sq {
                /*
                 * Obstacle viewed obliquely so that
                 * right vertex defines velocity obstacle.
                 */
                if !obstacle2.is_convex {
                    /* Ignore obstacle. */
                    // println!("1111111111111");
                    continue;
                }
                // println!("11111111111");
                {
                    obstacle1 = obstacle2;
                }
                

                let leg2 = (dist_sq2 - radius_sq).sqrt();
                println!("_left_leg_direction2");
                _left_leg_direction = Vector2::new(
                    relative_position2.x() * leg2 - relative_position2.y() * self.radius_,
                    relative_position2.x() * self.radius_ + relative_position2.y() * leg2,
                ) / dist_sq2;
                _right_leg_direction = Vector2::new(
                    relative_position2.x() * leg2 + relative_position2.y() * self.radius_,
                    -relative_position2.x() * self.radius_ + relative_position2.y() * leg2,
                ) / dist_sq2;
            } else {
                /* Usual situation. */
                if obstacle1.is_convex {
                    let leg1 = (dist_sq1 - radius_sq).sqrt();
                    println!("_left_leg_direction3");
                    _left_leg_direction = Vector2::new(
                        relative_position1.x() * leg1 - relative_position1.y() * self.radius_,
                        relative_position1.x() * self.radius_ + relative_position1.y() * leg1,
                    ) / dist_sq1;
                } else {
                    /* Left vertex non-convex; left leg extends cut-off line. */
                    println!("_left_leg_direction4");
                    _left_leg_direction = -obstacle1.unit_dir;
                }

                if obstacle2.is_convex {
                    let leg2 = (dist_sq2 - radius_sq).sqrt();
                    _right_leg_direction = Vector2::new(
                        relative_position2.x() * leg2 + relative_position2.y() * self.radius_,
                        -relative_position2.x() * self.radius_ + relative_position2.y() * leg2,
                    ) / dist_sq2;
                } else {
                    /* Right vertex non-convex; right leg extends cut-off line. */
                    _right_leg_direction = obstacle1.unit_dir;
                }
            }

            let left_neighbor = unsafe { &*obstacle1.prev_obstacle };
            if self.id_.0 == 0 {
                println!("left_neighbor: {:?}, obstacle1.prev_obstacle: {:?}", left_neighbor, obstacle1.prev_obstacle);
            }
            
            let mut is_left_leg_foreign = false;
            let mut is_right_leg_foreign = false;

            if self.id_.0 == 0 {
                println!("_left_leg_direction: {:?}, left_neighbor.unit_dir: {:?}, obstacle1.is_convex: {}, Vector2::det(&_left_leg_direction, &-left_neighbor.unit_dir): {}", 
                _left_leg_direction,  
                left_neighbor.unit_dir, 
                obstacle1.is_convex, 
                Vector2::det(&_left_leg_direction, &-left_neighbor.unit_dir));
            }
            if obstacle1.is_convex
                && Vector2::det(&_left_leg_direction, &-left_neighbor.unit_dir) >= 0.0
            {
                /* Left leg points into obstacle. */
                
                println!("_left_leg_direction5");
                _left_leg_direction = -left_neighbor.unit_dir;
                is_left_leg_foreign = true;
            }

            if obstacle2.is_convex
                && Vector2::det(&_right_leg_direction, &obstacle2.unit_dir) <= 0.0
            {
                /* Right leg points into obstacle. */
                _right_leg_direction = obstacle2.unit_dir;
                is_right_leg_foreign = true;
            }

            /* Compute cut-off centers. */
            let left_cutoff = (obstacle1.point_ - self.position_) * inv_time_horizon_obst;
            let right_cutoff = (obstacle2.point_ - self.position_) * inv_time_horizon_obst;
            let cutoff_vec = right_cutoff - left_cutoff;

            /* Project current velocity on velocity obstacle. */

            /* Check if current velocity is projected on cutoff circles. */
            let t = if obstacle1.id_ == obstacle2.id_ {
                0.5
            } else {
                if self.id_.0 == 0 {
                    println!("left_cutoff:{:?},  cutoff_vec:{:?}, self.velocity_:{:?}, Vector2::abs_sq(&cutoff_vec):{}", left_cutoff,  cutoff_vec, self.velocity_, Vector2::abs_sq(&cutoff_vec));
                    println!(
                        "(self.velocity_ - left_cutoff) * cutoff_vec): {}",
                        (self.velocity_ - left_cutoff) * cutoff_vec
                    );
                }
                ((self.velocity_ - left_cutoff) * cutoff_vec) / Vector2::abs_sq(&cutoff_vec)
            };
            let t_left = (self.velocity_ - left_cutoff) * _left_leg_direction;
            let t_right = (self.velocity_ - right_cutoff) * _right_leg_direction;
            if self.id_.0 == 0 {

                println!(
                    "t:{}, t_left:{}, t_right:{}, obstacle1.id: {}, obstacle2.id: {}",
                    t, t_left, t_right, obstacle1.id_, obstacle2.id_
                );

                println!(
                    "left_cutoff:{:?},  right_cutoff:{:?}, self.velocity_:{:?}, _left_leg_direction:{:?}, _right_leg_direction:{:?}",
                    left_cutoff,  right_cutoff, self.velocity_, _left_leg_direction, _right_leg_direction
                );
            }

            if (t < 0.0 && t_left < 0.0)
                || (obstacle1.id_ == obstacle2.id_ && t_left < 0.0 && t_right < 0.0)
            {
                /* Project on left cut-off circle. */
                let unit_w = Vector2::normalize(&(self.velocity_ - left_cutoff));

                line.direction = Vector2::new(unit_w.y(), -unit_w.x());
                line.point = left_cutoff + unit_w * (self.radius_ * inv_time_horizon_obst);
                if self.id_.0 == 0 {
                    println!("line33333333333");
                }
                self.orca_lines.push(line);
                continue;
            } else if t > 1.0 && t_right < 0.0 {
                /* Project on right cut-off circle. */
                let unit_w = Vector2::normalize(&(self.velocity_ - right_cutoff));

                line.direction = Vector2::new(unit_w.y(), -unit_w.x());
                line.point = right_cutoff + unit_w * self.radius_ * inv_time_horizon_obst;
                if self.id_.0 == 0 {
                    println!("line44444444444");
                }
                self.orca_lines.push(line);

                continue;
            }

            /*
             * Project on left leg, right leg, or cut-off line, whichever is closest
             * to velocity.
             */
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
            // if self.id_.0 == 0 {
            //     println!(
            //         "dist_sq_cutoff:{}, dist_sq_left:{}, dist_sq_right:{}",
            //         dist_sq_cutoff, dist_sq_left, dist_sq_right
            //     );
            // }
            if dist_sq_cutoff <= dist_sq_left && dist_sq_cutoff <= dist_sq_right {
                /* Project on cut-off line. */
                line.direction = -obstacle1.unit_dir;
                line.point = left_cutoff
                    + Vector2::new(-line.direction.y(), line.direction.x())
                        * self.radius_
                        * inv_time_horizon_obst;
                if self.id_.0 == 0 {
                    println!("line55555555555");
                }
                self.orca_lines.push(line);
                continue;
            } else if dist_sq_left <= dist_sq_right {
                /* Project on left leg. */
                if is_left_leg_foreign {
                    continue;
                }

                line.direction = _left_leg_direction;
                line.point = left_cutoff
                    + Vector2::new(-line.direction.y(), line.direction.x())
                        * self.radius_
                        * inv_time_horizon_obst;
                if self.id_.0 == 0 {
                    println!("line6666666666");
                }
                self.orca_lines.push(line);
                continue;
            } else {
                /* Project on right leg. */
                if is_right_leg_foreign {
                    continue;
                }

                line.direction = -_right_leg_direction;
                line.point = right_cutoff
                    + Vector2::new(-line.direction.y(), line.direction.x())
                        * self.radius_
                        * inv_time_horizon_obst;
                if self.id_.0 == 0 {
                    println!("line7777777777777");
                }
                self.orca_lines.push(line);
                continue;
            }
        }

        let num_obst_lines = self.orca_lines.len();

        let inv_time_horizon = 1.0 / self.time_horizon;

        /* Create agent ORCA lines. */
        for i in 0..self.agent_neighbors.len() {
            let other = unsafe { &*(self.agent_neighbors.get_mut(i).unwrap().1) };

            let relative_position = other.position_ - self.position_;
            // println!("other.position_ : {:?},  self.position_: {:?}", other.position_,  self.position_);
            let relative_velocity = self.velocity_ - other.velocity_;
            // println!("self.velocity_ : {:?},  other.velocity_: {:?}", self.velocity_,  other.velocity_);
            let dist_sq = Vector2::abs_sq(&relative_position);
            let combined_radius = self.radius_ + other.radius_;
            let combined_radius_sq = Vector2::sqr(combined_radius);

            let mut line = Line::default();
            let mut _u: Vector2 = Vector2::default();

            if dist_sq > combined_radius_sq {
                /* No collision. */
                let w = relative_velocity - relative_position * inv_time_horizon;
                /* Vector from cutoff center to relative velocity. */
                let w_length_sq = Vector2::abs_sq(&w);

                let dot_product1 = w * relative_position;

                if dot_product1 < 0.0
                    && Vector2::sqr(dot_product1) > combined_radius_sq * w_length_sq
                {
                    /* Project on cut-off circle. */
                    let w_length = w_length_sq.sqrt();
                    let unit_w = w / w_length;

                    line.direction = Vector2::new(unit_w.y(), -unit_w.x());
                    // println!(" combinedRadius: {:?}, invTimeHorizon: {:?}, wLength: {:?}, unitW : {:?},", combinedRadius, invTimeHorizon, wLength, unitW);
                    _u = unit_w * (combined_radius * inv_time_horizon - w_length);
                } else {
                    /* Project on legs. */
                    let leg = (dist_sq - combined_radius_sq).sqrt();

                    if Vector2::det(&relative_position, &w) > 0.0 {
                        /* Project on left leg. */
                        line.direction = Vector2::new(
                            relative_position.x() * leg - relative_position.y() * combined_radius,
                            relative_position.x() * combined_radius + relative_position.y() * leg,
                        ) / dist_sq;
                    } else {
                        /* Project on right leg. */
                        line.direction = -Vector2::new(
                            relative_position.x() * leg + relative_position.y() * combined_radius,
                            -relative_position.x() * combined_radius + relative_position.y() * leg,
                        ) / dist_sq;
                    }

                    let dot_product2 = relative_velocity * line.direction;
                    // // println!("line.direction : {:?}, dotProduct2: {:?}, relativeVelocity: {:?}", line.direction, dotProduct2, relativeVelocity);
                    _u = line.direction * dot_product2 - relative_velocity;
                }
            } else {
                /* Collision. Project on cut-off circle of time timeStep. */
                let inv_time_step = 1.0 / (unsafe { &*self.sim_ }).time_step;

                /* Vector from cutoff center to relative velocity. */
                let w = relative_velocity - relative_position * inv_time_step;

                let w_length = Vector2::abs(&w);
                let unit_w = w / w_length;

                line.direction = Vector2::new(unit_w.y(), -unit_w.x());
                // // println!("unitW : {:?}, combinedRadius: {:?}, invTimeStep: {:?}, wLength: {:?}", unitW, combinedRadius, invTimeStep, wLength);
                _u = unit_w * (combined_radius * inv_time_step - w_length);
            }
            // // println!("elf.velocity_: {:?}, u: {:?}, self.velocity_ + u * 0.5: {:?}", self.velocity_, u, self.velocity_ + u * 0.5);
            line.point = self.velocity_ + _u * 0.5;
            if self.id_.0 == 0 {
                println!("line88888888888");
            }
            self.orca_lines.push(line);
        }
        if self.id_.0 == 0 {
            println!("linearProgram0: self.position_: {:?}", self.position_);
            println!("linearProgram0: self.orca_lines: {:?}", self.orca_lines);
            println!(
                "linearProgram0: self.pref_velocity: {:?}",
                self.pref_velocity
            );
            println!("linearProgram0: self.max_speed: {:?}", self.max_speed);
            println!("linearProgram0: self.new_velocity: {:?}", self.new_velocity);
        }

        if self.id_.0 == 0 {
            println!(" self.new_velocity0: {:?}", self.new_velocity);
        }

        let line_fail = Self::linear_program2(
            &self.orca_lines,
            self.max_speed,
            &self.pref_velocity,
            false,
            &mut self.new_velocity,
        );

        if self.id_.0 == 0 {
            println!("linearProgram1: lineFail {:?}", line_fail);
            println!(" self.new_velocity1: {:?}", self.new_velocity);
            println!("num_obst_lines : {}", num_obst_lines);
        }
        if line_fail < self.orca_lines.len() {
            Self::linear_program3(
                &self.orca_lines,
                num_obst_lines,
                line_fail,
                self.max_speed,
                &mut self.new_velocity,
                self.id_.0,
            );
        }
        if self.id_.0 == 0 {
            println!(" self.new_velocity2: {:?}", self.new_velocity);
        }
    }

    pub fn insert_agent_neighbor(&mut self, agent: &mut Agent, range_sq: &mut f32) {
        if self as *const Agent != agent {
            let dist_sq = Vector2::abs_sq(&(self.position_ - agent.position_));

            if dist_sq < *range_sq {
                if self.agent_neighbors.len() < self.max_neighbors {
                    self.agent_neighbors.push((dist_sq, agent));
                }

                let mut i = self.agent_neighbors.len() - 1;

                while i != 0 && dist_sq < self.agent_neighbors[i - 1].0 {
                    self.agent_neighbors[i] = self.agent_neighbors[i - 1].clone();
                    i -= 1;
                }

                self.agent_neighbors[i] = (dist_sq, agent);

                if self.agent_neighbors.len() == self.max_neighbors {
                    *range_sq = self.agent_neighbors.last().unwrap().0;
                }
            }
        }
    }

    pub fn insert_obstacle_neighbor(&mut self, obstacle: Obstacle, range_sq: f32) {
        let next_obstacle = unsafe { &*obstacle.next_obstacle };

        let dist_sq = Vector2::dist_sq_point_line_segment(
            &obstacle.point_,
            &next_obstacle.point_,
            &self.position_,
        );

        // if self.id_.0 == 0 {
        //     println!("obstacle.point_: {:?}", obstacle.point_);
        //     println!("next_obstacle.point_: {:?}", next_obstacle.point_);
        //     println!("self.position_: {:?}", self.position_);
        //     println!("range_sq: {:?}, dist_sq: {:?}", range_sq, dist_sq);
        // }
        
        if dist_sq < range_sq {
            self.obstacle_neighbors.push((dist_sq, obstacle.clone()));

            let mut i = self.obstacle_neighbors.len() - 1;

            while i != 0 && dist_sq < self.obstacle_neighbors[i - 1].0 {
                self.obstacle_neighbors[i] = self.obstacle_neighbors[i - 1];
                i -= 1;
            }

            self.obstacle_neighbors[i] = (dist_sq, obstacle);
        }
        
        

    }

    pub fn update(&mut self) {
        self.velocity_ = self.new_velocity;
        // println!("agent{} velocity: {:?}",self.id_.0, self.velocity_);
        // // println!(
        //     "(unsafe  &*self.sim_ ).timeStep_: {:?}",
        //     (unsafe { &*self.sim_ }).timeStep_
        // );
        self.position_ = self.position_ + self.velocity_ * (unsafe { &*self.sim_ }).time_step;
        // println!("self.position_: {:?}", self.position_);
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
            /* Max speed circle fully invalidates line lineNo. */
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
                /* Lines lineNo and i are (almost) parallel. */
                if numerator < 0.0 {
                    return false;
                } else {
                    continue;
                }
            }

            let t = numerator / denominator;

            if denominator >= 0.0 {
                /* Line i bounds line lineNo on the right. */
                t_right = t_right.min(t);
            } else {
                /* Line i bounds line lineNo on the left. */
                t_left = t.max(t_left);
            }

            if t_left > t_right {
                return false;
            }
        }
        // println!("linearProgram1: result0: {:?}", result);
        if direction_opt {
            /* Optimize direction. */
            if lines[line_no].direction * opt_velocity > 0.0 {
                /* Take right extreme. */
                *result = lines[line_no].point + lines[line_no].direction * t_right;
                // println!("linearProgram1: result1: {:?}", result);
            } else {
                /* Take left extreme. */
                *result = lines[line_no].point + lines[line_no].direction * t_left;
                // println!("linearProgram1: result2: {:?}", result);
            }
        } else {
            /* Optimize closest point. */
            let t = lines[line_no].direction * (*opt_velocity - lines[line_no].point);
            // // println!("linearProgram1: t: {:?}, tLeft: {:?}, tRight: {:?}", t, tLeft, tRight);
            if t < t_left {
                *result = lines[line_no].point + lines[line_no].direction * t_left;
                // println!("linearProgram1: result3: {:?}", result);
            } else if t > t_right {
                *result = lines[line_no].point + lines[line_no].direction * t_right;
                // println!("linearProgram1: result4: {:?}", result);
            } else {
                *result = lines[line_no].point + lines[line_no].direction * t;
                // println!("linearProgram1: result5: {:?}", result);
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
             * Optimize direction. Note that the optimization velocity is of unit
             * length in this case.
             */
            *result = *opt_velocity * radius;
            // println!("1result: {:?}", result);
        } else if Vector2::abs_sq(opt_velocity) > Vector2::sqr(radius) {
            /* Optimize closest point and outside circle. */
            *result = Vector2::normalize(opt_velocity) * radius;
            // println!("2result: {:?}", result);
        } else {
            /* Optimize closest point and inside circle. */
            *result = *opt_velocity;
            // println!("3result: {:?}", result);
        }

        for i in 0..lines.len() {
            // println!("lines[i].direction: {:?}, ines[i].point: {:?}, result: {:?}", lines[i].direction, lines[i].point, result);
            let r = Vector2::det(&lines[i].direction, &(lines[i].point - *result));
            // println!("r: {:?}", r);
            if (r) > 0.0 {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                let temp_result = *result;
                // println!("4result: {:?}", result);
                if !Self::linear_program1(lines, i, radius, opt_velocity, direction_opt, result) {
                    *result = temp_result;
                    // println!("5result: {:?}", result);
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
        id: usize,
    ) {
        let mut distance = 0.0;

        for i in begin_line..lines.len() {
            if Vector2::det(&lines[i].direction, &(lines[i].point - *result)) > distance {
                /* Result does not satisfy constraint of line i. */
                // std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));
                let mut proj_lines = vec![];

                lines[0..num_obst_lines].iter().for_each(|l|{
                    proj_lines.push(l.clone());
                });
                

                for j in num_obst_lines..i {
                    let mut line = Line::default();

                    let determinant = Vector2::det(&lines[i].direction, &lines[j].direction);

                    if determinant.abs() <= RVO_EPSILON {
                        /* Line i and line j are parallel. */
                        if lines[i].direction * lines[j].direction > 0.0 {
                            /* Line i and line j point in the same direction. */
                            continue;
                        } else {
                            /* Line i and line j point in opposite direction. */
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
                    // if id ==0{
                    //     println!("i: {}, j: {}, begin_line: {:?}, lines.len(): {}, num_obst_lines: {}", i, j, begin_line, lines.len(), num_obst_lines);
                    // }
                    
                    line.direction = Vector2::normalize(&(lines[j].direction - lines[i].direction));
                    proj_lines.push(line);
                }

                let temp_result = *result;

                // if id == 0 {
                //     println!("proj_lines: {:?}, radius: {}, result: {:?}, Vector2::new(-lines[i].direction.y(), lines[i].direction.x()): {:?}", 
                //     proj_lines, 
                //     radius, 
                //     result, 
                //     Vector2::new(-lines[i].direction.y(), lines[i].direction.x()));
                // }

                if Self::linear_program2(
                    &proj_lines,
                    radius,
                    &Vector2::new(-lines[i].direction.y(), lines[i].direction.x()),
                    true,
                    result,
                ) < proj_lines.len()
                {
                    /* This should in principle not happen.  The result is by definition
                     * already in the feasible region of this linear program. If it fails,
                     * it is due to small floating point error, and the current result is
                     * kept.
                     */
                    if id == 0 {
                        // println!("linear_program2 failed");
                    }
                    *result = temp_result;
                }
                if id == 0 {
                // println!("result: {:?}", result);
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

    pub fn get_agent_obstacle_neighbor(&self, neighbor_no: usize) -> usize {
        return self.obstacle_neighbors[neighbor_no].1.id_;
    }

    pub fn get_agent_orcaline(&self, line_no: usize) -> Line {
        return self.orca_lines[line_no];
    }

    pub fn get_agent_agent_neighbor(&self, neighbor_no: usize) -> usize {
        return (unsafe { &*(self.agent_neighbors[neighbor_no].1) }).id_.0;
    }
}

impl Agent {
    pub fn compute_aabb(&self) -> AABB<f32> {
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
}

// 判断直线p1p2与圆c是否相交，相交返回true，否则返回false
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
