use pi_orca::{rvos_imulator::RVOSimulator, util::Vertices, vector2::Vector2};
// use rand::Rng;

// const temp: f32 = 200.;
// const num: usize = 100;
// const RAND_MAX: f32 = 32767.;
// const RVO_TWO_PI: f32 = 6.28318530717958647692;
fn main() {
    let mut sim = RVOSimulator::default(2000);
    let mut goals = vec![];
    let mut agents = vec![];

    setup_scenario(&mut sim, &mut goals, &mut agents);
    let begin = std::time::Instant::now();
    let mut n = 0;
    loop {
        if n > 2000{
            break;
        }
        update_visualization(&mut sim, &agents);

        set_preferred_velocities(&mut sim, &goals, &agents);
        sim.do_step();

        if reached_goal(&mut sim, &goals, &agents) {
            // break;
        }

        // std::thread::sleep(std::time::Duration::from_millis(16));
        n+=1;
    }
    println!("time = {:?}", begin.elapsed());
}

pub fn setup_scenario(sim: &mut RVOSimulator, goals: &mut Vec<Vector2>, agents: &mut Vec<f64>) {
    /* Specify the global time step of the simulation. */
    sim.set_time_step(0.25);

    /* Specify the default parameters for agents that are subsequently added. */
    sim.set_agent_defaults(15.0, 100, 5.0, 5.0, 2.0, 2.0, &Vector2::default());

    // let mut id = 0;
    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */
    for i in 0..5 {
        for j in 0..5 {
            let pos = Vector2::new(55.0 + i as f32 * 10.0, 55.0 + j as f32 * 10.0);
            let id = sim.add_agent(&pos);
            agents.push(id);
            goals.push(-pos);

            let pos = Vector2::new(-55.0 - i as f32 * 10.0, 55.0 + j as f32 * 10.0);
            let id = sim.add_agent(&pos);
            agents.push(id);
            goals.push(-pos);

            let pos = Vector2::new(55.0 + i as f32 * 10.0, -55.0 - j as f32 * 10.0);
            let id = sim.add_agent(&pos);
            agents.push(id);
            goals.push(-pos);

            let pos = Vector2::new(-55.0 - i as f32 * 10.0, -55.0 - j as f32 * 10.0);
            let id = sim.add_agent(&pos);
            agents.push(id);
            goals.push(-pos);
        }
    }

    // println!("sim.remove_agent(id) r: {}", sim.remove_agent(id));
    /*
     * Add (polygonal) obstacles, specifying their vertices in counterclockwise
     * order.
     */
    let mut obstacle1 = Vertices::new();
    let mut obstacle2 = Vertices::new();
    let mut obstacle3 = Vertices::new();
    let mut obstacle4 = Vertices::new();

    obstacle1.add(Vector2::new(-10.0, 40.0));
    obstacle1.add(Vector2::new(-40.0, 40.0));
    obstacle1.add(Vector2::new(-40.0, 10.0));
    obstacle1.add(Vector2::new(-10.0, 10.0));

    obstacle2.add(Vector2::new(10.0, 40.0));
    obstacle2.add(Vector2::new(10.0, 10.0));
    obstacle2.add(Vector2::new(40.0, 10.0));
    obstacle2.add(Vector2::new(40.0, 40.0));

    obstacle3.add(Vector2::new(10.0, -40.0));
    obstacle3.add(Vector2::new(40.0, -40.0));
    obstacle3.add(Vector2::new(40.0, -10.0));
    obstacle3.add(Vector2::new(10.0, -10.0));

    obstacle4.add(Vector2::new(-10.0, -40.0));
    obstacle4.add(Vector2::new(-10.0, -10.0));
    obstacle4.add(Vector2::new(-40.0, -10.0));
    obstacle4.add(Vector2::new(-40.0, -40.0));

    sim.add_obstacle(obstacle1);
    sim.add_obstacle(obstacle2);
    sim.add_obstacle(obstacle3);
    sim.add_obstacle(obstacle4);
}

pub fn update_visualization(sim: &mut RVOSimulator, agents: &Vec<f64>) {
    // println!("global_time : {}", sim.get_global_time());

    // for id in agents {
    //     print!(" {:?}", sim.get_agent_position(*id));
    // }
    // println!("");
}

pub fn set_preferred_velocities(sim: &mut RVOSimulator, goals: &Vec<Vector2>, agents: &Vec<f64>) {
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */

    for (id, goal) in agents.iter().zip(goals.iter()) {
        let mut goal_vector = *goal - sim.get_agent_position(*id).unwrap();

        if Vector2::abs_sq(&goal_vector) > 1.0 {
            goal_vector = Vector2::normalize(&goal_vector);
        }
        // println!("goalVector : {:?}", goalVector);
        sim.set_agent_pref_velocity(*id, &goal_vector);
    }
}

pub fn reached_goal(sim: &mut RVOSimulator, goals: &Vec<Vector2>, agents: &Vec<f64>) -> bool {
    /* Check if all agents have reached their goals. */
    for (goal, id) in goals.iter().zip(agents) {
        if Vector2::abs_sq(&(sim.get_agent_position(*id).unwrap() - goal)) > 20.0 * 20.0 {
            return false;
        }
    }

    return true;
}
