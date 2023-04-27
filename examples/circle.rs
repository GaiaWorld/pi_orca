use pi_orca::{
    obstacle::Vertices,
    rvos_imulator::{RVOSimulator, ID},
    vector2::Vector2,
};

const temp: f32 = 200.;
const num: usize = 100;
const RAND_MAX: f32 = 32767.;

use rand::Rng;
fn main() {
    let mut sim = RVOSimulator::default(2000);
    let mut goals = vec![];
    let mut agents = vec![];
    let mut rng = rand::thread_rng();
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

        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}

pub fn setup_scenario(sim: &mut RVOSimulator, goals: &mut Vec<Vector2>, agents: &mut Vec<f64>) {
    sim.set_time_step(0.25);

    sim.set_agent_defaults(15.0, 10, 10.0, 10.0, 1.5, 2.0, &Vector2::default());

    // for i in 0..num {
    //     let pos = Vector2::new(
    //         (i as f32 * 2.0 * std::f32::consts::PI / num as f32).cos(),
    //         (i as f32 * 2.0 * std::f32::consts::PI / num as f32).sin(),
    //     ) * temp;
    //     sim.add_agent(&pos);
    //     // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
    //     goals.push(-sim.get_agent_position(i));
    // }
    agents.push(sim.add_agent(&Vector2::new(0., 0.)));
    // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
    goals.push(Vector2::new(100., 100.));

    agents.push(sim.add_agent(&Vector2::new(100., 0.)));
    // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
    goals.push(Vector2::new(0., 100.));

    agents.push(sim.add_agent(&Vector2::new(0., 100.)));
    // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
    goals.push(Vector2::new(100., 0.));

    agents.push(sim.add_agent(&Vector2::new(100., 100.)));
    // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
    goals.push(Vector2::new(0., 0.));

    let mut obstacle1 = Vertices::new();

    obstacle1.add(Vector2::new(60.0, 60.0));
    obstacle1.add(Vector2::new(40.0, 60.0));
    obstacle1.add(Vector2::new(40.0, 40.0));
    obstacle1.add(Vector2::new(60.0, 40.0));

    sim.add_obstacle(obstacle1);
}

pub fn update_visualization(sim: &mut RVOSimulator, agents: &Vec<f64>) {
    print!("{}", sim.get_global_time());

    for id in agents {
        print!(" {:?}", sim.get_agent_position(*id));
    }
    println!("");
}

pub fn set_preferred_velocities(sim: &mut RVOSimulator, goals: &Vec<Vector2>, agents: &Vec<f64>) {
    let mut rng = rand::thread_rng();

    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
    for (id, goal) in agents.iter().zip(goals) {
        let mut goal_vector = *goal - sim.get_agent_position(*id).unwrap();
        // println!("setAgentPrefVelocity0: {:?}", goal_vector);
        // println!("Vector2::absSq(&goal_vector): {:?}", Vector2::absSq(&goal_vector));
        // println!("Vector2::normalize(&goal_vector): {:?}", Vector2::normalize(&goal_vector));
        if Vector2::abs_sq(&goal_vector) > 1.0 {
            goal_vector = Vector2::normalize(&goal_vector);
        }
        // println!("setAgentPrefVelocity: {:?}", goal_vector);
        sim.set_agent_pref_velocity(*id, &goal_vector);

        let angle = rng.gen::<f32>() * RAND_MAX * 2.0 * std::f32::consts::PI / RAND_MAX;
        let dist = rng.gen::<f32>() * RAND_MAX * 0.0001 / RAND_MAX;
        // println!("angle: {:?}, dist: {:?}", angle, dist);
        let v = sim.get_agent_pref_velocity(*id).unwrap()
            + Vector2::new(angle.cos(), angle.sin()) * dist;

        // println!(
        //     "Vector2::new(angle.cos(), angle.sin()) * dist: {:?}",
        //     Vector2::new(angle.cos(), angle.sin()) * dist
        // );

        // println!("setAgentPrefVelocity: {:?}", v);
        sim.set_agent_pref_velocity(*id, &v);
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
