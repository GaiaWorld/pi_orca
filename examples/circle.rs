use pi_orca::{rvos_imulator::RVOSimulator, vector2::Vector2};

const temp: f32 = 200.;
const num: usize = 100;
fn main() {
    let mut sim = RVOSimulator::default();
    let mut goals = vec![];

    setup_scenario(&mut sim, &mut goals);

    loop {
        update_visualization(&mut sim);

        set_preferred_velocities(&mut sim, &goals);
        sim.do_step();

        if reached_goal(&mut sim, &goals) {
            break;
        }

        std::thread::sleep(std::time::Duration::from_millis(200));
    }
}

pub fn setup_scenario(sim: &mut RVOSimulator, goals: &mut Vec<Vector2>) {
    sim.set_time_step(0.25);

    sim.set_agent_defaults(15.0, 10, 10.0, 10.0, 1.5, 2.0, &Vector2::default());

    for i in 0..num {
        let pos = Vector2::new(
            (i as f32 * 2.0 * std::f32::consts::PI / num as f32).cos(),
            (i as f32 * 2.0 * std::f32::consts::PI / num as f32).sin(),
        ) * temp;
        sim.add_agent(&pos);
        // println!("-sim.getAgentPosition(i): {:?}", -sim.getAgentPosition(i));
        goals.push(-sim.get_agent_position(i));
    }
}

pub fn update_visualization(sim: &mut RVOSimulator) {
    // print!("{}", sim.getGlobalTime());

    // for i in 0..sim.getNumAgents() {
    //     print!(" {:?}", sim.getAgentPosition(i));
    // }
    // println!("");
}

pub fn set_preferred_velocities(sim: &mut RVOSimulator, goals: &Vec<Vector2>) {
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
    for i in 0..sim.get_num_agents() {
        let mut goal_vector = goals[i] - sim.get_agent_position(i);
        // println!("setAgentPrefVelocity0: {:?}", goal_vector);
        // println!("Vector2::absSq(&goal_vector): {:?}", Vector2::absSq(&goal_vector));
        // println!("Vector2::normalize(&goal_vector): {:?}", Vector2::normalize(&goal_vector));
        if Vector2::abs_sq(&goal_vector) > 1.0 {
            goal_vector = Vector2::normalize(&goal_vector);
        }
        // println!("setAgentPrefVelocity: {:?}", goal_vector);
        sim.set_agent_pref_velocity(i, &goal_vector);
    }
}

pub fn reached_goal(sim: &mut RVOSimulator, goals: &Vec<Vector2>) -> bool {
    /* Check if all agents have reached their goals. */
    for i in 0..sim.get_num_agents() {
        if Vector2::abs_sq(&(sim.get_agent_position(i) - goals[i]))
            > sim.get_agent_radius(i) * sim.get_agent_radius(i)
        {
            return false;
        }
    }

    return true;
}
