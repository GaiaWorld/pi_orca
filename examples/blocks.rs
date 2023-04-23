use pi_orca::{obstacle::Vertices, rvos_imulator::RVOSimulator, vector2::Vector2};
use rand::Rng;

const temp: f32 = 200.;
const num: usize = 100;
const RAND_MAX: f32 = 32767.;
const RVO_TWO_PI: f32 = 6.28318530717958647692;
fn main() {
    let mut sim = RVOSimulator::default();
    let mut goals = vec![];

    setup_scenario(&mut sim, &mut goals);

    loop {
        if sim.get_global_time() > 23.5 {
            break;
        }

        update_visualization(&mut sim);

        set_preferred_velocities(&mut sim, &goals);
        sim.do_step();

        if reached_goal(&mut sim, &goals) {
            break;
        }
        

        std::thread::sleep(std::time::Duration::from_millis(16));
    }
}

pub fn setup_scenario(sim: &mut RVOSimulator, goals: &mut Vec<Vector2>) {
    /* Specify the global time step of the simulation. */
    sim.set_time_step(0.25);

    /* Specify the default parameters for agents that are subsequently added. */
    sim.set_agent_defaults(15.0, 10, 5.0, 5.0, 2.0, 2.0, &Vector2::default());

    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */
    for i in 0..2 {
        for j in 0..2 {
            // sim.add_agent(&Vector2::new(
            //     55.0 + i as f32 * 10.0,
            //     55.0 + j as f32 * 10.0,
            // ));
            // goals.push(Vector2::new(-75.0, -75.0));

            sim.add_agent(&Vector2::new(
                -55.0 - i as f32 * 10.0,
                55.0 + j as f32 * 10.0,
            ));
            goals.push(Vector2::new(75.0, -75.0));

            // sim.add_agent(&Vector2::new(
            //     55.0 + i as f32 * 10.0,
            //     -55.0 - j as f32 * 10.0,
            // ));
            // goals.push(Vector2::new(-75.0, 75.0));

            // sim.add_agent(&Vector2::new(
            //     -55.0 - i as f32 * 10.0,
            //     -55.0 - j as f32 * 10.0,
            // ));
            // goals.push(Vector2::new(75.0, 75.0));
        }
    }

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
    // sim.add_obstacle(obstacle3);
    // sim.add_obstacle(obstacle4);

    /* Process the obstacles so that they are accounted for in the simulation. */
    // sim.processObstacles();
}

pub fn update_visualization(sim: &mut RVOSimulator) {
    println!("global_time : {}", sim.get_global_time());

    for i in 0..sim.get_num_agents() {
        print!(" {:?}", sim.get_agent_position(i));
    }
    println!("");
}

pub fn set_preferred_velocities(sim: &mut RVOSimulator, goals: &Vec<Vector2>) {
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
    let mut rng = rand::thread_rng();

    for i in 0..sim.get_num_agents() {
        let mut goalVector = goals[i] - sim.get_agent_position(i);

        if Vector2::abs_sq(&goalVector) > 1.0 {
            goalVector = Vector2::normalize(&goalVector);
        }
        // println!("goalVector : {:?}", goalVector);
        sim.set_agent_pref_velocity(i, &goalVector);

        /*
         * Perturb a little to avoid deadlocks due to perfect symmetry.
         */
        // let angle = rng.gen::<f32>() * RAND_MAX * 2.0 * std::f32::consts::PI / RAND_MAX;
        // let dist = rng.gen::<f32>() * RAND_MAX * 0.0001 / RAND_MAX;

        // let angle = 1.5;
        // let dist = 1.5;
        // println!("angle : {}, dist : {}, rng.gen::<f32>(): {}", angle, dist, rng.gen::<f32>());
        // sim.set_agent_pref_velocity(
        //     i,
        //     &(sim.get_agent_pref_velocity(i) + Vector2::new(angle.cos(), angle.sin()) * dist),
        // );
    }
}

pub fn reached_goal(sim: &mut RVOSimulator, goals: &Vec<Vector2>) -> bool {
    /* Check if all agents have reached their goals. */
    for i in 0..sim.get_num_agents() {
        if Vector2::abs_sq(&(sim.get_agent_position(i) - goals[i])) > 20.0 * 20.0 {
            return false;
        }
    }

    return true;
}
