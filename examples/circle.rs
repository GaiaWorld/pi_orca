use pi_orca::{rvos_imulator::RVOSimulator, vector2::Vector2};

const TEMP: f32 = 20.;
const NUM: usize = 4;

fn main() {
    let mut sim = RVOSimulator::default();
    let mut goals = vec![];
    let mut agents = vec![];
    // for i in 0..100 {
    //     println!("{}: {}", i, rng.gen::<f32>());
    // }

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

        if reached_goal(&mut sim, &goals, &agents) {
            break;
        }

        std::thread::sleep(std::time::Duration::from_millis(16));
    }
}

pub fn setup_scenario(sim: &mut RVOSimulator, goals: &mut Vec<Vector2>, agents: &mut Vec<f64>) {
    sim.set_time_step(0.25);

    sim.set_agent_defaults(15.0, 10, 10.0, 10.0, 1.5, 2.0, &Vector2::default());

    for i in 0..NUM {
        let pos = Vector2::new(
            (i as f32 * 2.0 * std::f32::consts::PI / NUM as f32).cos(),
            (i as f32 * 2.0 * std::f32::consts::PI / NUM as f32).sin(),
        ) * TEMP;

        agents.push(sim.add_agent(&Vector2::new(190., 190.)));
        // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
        goals.push(-sim.get_agent_position(agents[i]).unwrap());
    }

    // let pos = Vector2::new(-1., -1.) * temp;

    // agents.push(sim.add_agent(&pos));
    // goals.push(-sim.get_agent_position(agents[0]).unwrap());

    // let pos = Vector2::new(1., -1.) * temp;

    // agents.push(sim.add_agent(&pos));
    // goals.push(-sim.get_agent_position(agents[1]).unwrap());
}

pub fn update_visualization(sim: &mut RVOSimulator, agents: &Vec<f64>) {
    println!("{}", sim.get_global_time());

    for id in agents {
        println!(" {:?}", sim.get_agent_position(*id));
    }
    println!("");
}

pub fn set_preferred_velocities(sim: &mut RVOSimulator, goals: &Vec<Vector2>, agents: &Vec<f64>) {
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
    for (id, goal) in agents.iter().zip(goals) {
        let mut goal_vector = *goal - sim.get_agent_position(*id).unwrap();
        if Vector2::abs_sq(&goal_vector) > 1.0 {
            goal_vector = Vector2::normalize(&goal_vector);
        }
        // println!("setAgentPrefVelocity: {:?}", goal_vector);
        sim.set_agent_pref_velocity(*id, &goal_vector);
    }
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

    return true;
}
