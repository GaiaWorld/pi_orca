use pi_orca::{rvos_imulator::RVOSimulator, vector2::Vector2};
use rand::Rng;
use rand_core::SeedableRng;

const TEMP: f32 = 20.;
const NUM: usize = 4;
const MAX_SPEED: f32 = 0.5;
fn main() {
    let mut sim = RVOSimulator::default();
    let mut goals = vec![];
    let mut agents = vec![];
    // for i in 0..100 {
    //     println!("{}: {}", i, rng.gen::<f32>());
    // }
    // let dist = sim.get_rand() * 0.0001;
    // self.velocity_ = self.pref_velocity - Vector2::new(angle.cos(), angle.sin()) * dist;

    setup_scenario(&mut sim, &mut goals, &mut agents);
    for agent in &agents {
        println!("agent: {:?}", unsafe {
            std::mem::transmute::<f64, u64>(*agent)
        });
    }

    loop {
        update_visualization(&mut sim, &agents);

        set_preferred_velocities(&mut sim, &goals, &agents);
        sim.do_step();

        // if reached_goal(&mut sim, &goals, &agents) {
        //     break;
        // }

        std::thread::sleep(std::time::Duration::from_millis(16));
    }
}

pub fn setup_scenario(sim: &mut RVOSimulator, goals: &mut Vec<Vector2>, agents: &mut Vec<f64>) {
    sim.set_time_step(0.25);

    sim.set_agent_defaults(1.5, 10, 10., 10., 0.15, MAX_SPEED, &Vector2::default());

    // for i in 0..NUM {
    //     let pos = Vector2::new(
    //         (i as f32 * 2.0 * std::f32::consts::PI / NUM as f32).cos(),
    //         (i as f32 * 2.0 * std::f32::consts::PI / NUM as f32).sin(),
    //     ) * TEMP;

    //     agents.push(sim.add_agent(&pos, 0.5));
    //     // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
    //     sim.set_agent_goal(agents[i], Some(-pos));
    // }

    // let pos = Vector2::new(100., 0.);

    agents.push(sim.add_agent(&Vector2::new(20.5, 17.5), MAX_SPEED));
    sim.set_agent_goal(agents[0], Some(Vector2::new(22.0, 11.0)));
    // sim.set_agent_velocity(agents[0], &Vector2::new(-5., 0.0));

    agents.push(sim.add_agent(&Vector2::new(21.5, 17.5), MAX_SPEED));
    sim.set_agent_goal(agents[1], Some(Vector2::new(22.0, 11.0)));

    agents.push(sim.add_agent(&Vector2::new(22.6, 17.5), MAX_SPEED));
    sim.set_agent_goal(agents[2], Some(Vector2::new(22.0, 11.0)));

    // sim.set_agent_velocity(agents[1], &Vector2::new(0.000001029349050440942, 0.0000006012704147906334));

    // agents.push(sim.add_agent(&Vector2::new(20.984811782836914, 5.300600051879883), 0.5));
    // sim.set_agent_goal(agents[2], Some(Vector2::new(21.0, 5.0)));

    // let pos = Vector2::new(90., 0.);
    // agents.push(sim.add_agent(&pos, 2.0));
    // sim.set_agent_goal(agents[1], Some(Vector2::new(20., 0.)));
}

pub fn update_visualization(sim: &mut RVOSimulator, agents: &Vec<f64>) {
    println!("{}", sim.get_global_time());
    // let mut r = true;
    for id in agents {
        let pos = sim.get_agent_position(*id);
        println!("当前位置： {:?}", pos);

        let goal = sim.get_agent_goal(*id);
        println!("期望目标: {:?}", goal);

        let dist = Vector2::abs(&(goal.unwrap() - pos.unwrap()));
        println!("目标距离: {:?}", dist);

        let velocity = sim.get_agent_velocity(*id);
        println!("当前速度：{:?}", velocity);

        let pref_velocity = sim.get_agent_pref_velocity(*id);
        println!("期望速度: {:?}", pref_velocity);

        let num = sim.get_agent_num_orcalines(*id).unwrap();
        println!("超平面数量: {:?}", num);

        for i in 0..num {
            let orca = sim.get_agent_orcaline(*id, i);
            println!("超平面: {:?}", orca);
        }

        if let Some(goal) = sim.get_agent_goal(*id) {
            let pos = pos.unwrap();
            if goal.x == pos.x && goal.y == pos.y {
                // r = false;
                println!("到达目标");
                sim.set_agent_goal(*id, None);
            }
        }
    }
    println!("");
}

pub fn set_preferred_velocities(sim: &mut RVOSimulator, goals: &Vec<Vector2>, agents: &Vec<f64>) {
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
    // for (id, goal) in agents.iter().zip(goals) {
    //     let mut goal_vector = *goal - sim.get_agent_position(*id).unwrap();
    //     if Vector2::abs_sq(&goal_vector) > 1.0 {
    //         goal_vector = Vector2::normalize(&goal_vector);
    //     }
    //     // println!("setAgentPrefVelocity: {:?}", goal_vector);
    //     sim.set_agent_pref_velocity(*id, &goal_vector);
    // }
}

pub fn reached_goal(sim: &mut RVOSimulator, goals: &Vec<Vector2>, agents: &Vec<f64>) -> bool {
    /* Check if all agents have reached their goals. */
    for (id, goal) in agents.iter().zip(goals) {
        if Vector2::abs_sq(&(sim.get_agent_position(*id).unwrap() - goal))
            > sim.get_agent_radius(*id).unwrap() * sim.get_agent_radius(*id).unwrap()
        {
            return false;
        }
    }

    // return true;
    true
}
