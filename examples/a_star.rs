use pi_orca::{
    a_start::{AStar, NodeIndex, ResultPath, TileMap, TileObstacle},
    obstacle::Vertices,
    rvos_imulator::RVOSimulator,
    vector2::Vector2,
};
use rand::Rng;

const temp: f32 = 200.;
const num: usize = 100;
const RAND_MAX: f32 = 32767.;
const RVO_TWO_PI: f32 = 6.28318530717958647692;
const map_width: usize = 11;
const map_height: usize = 11;

const agent_row: usize = 1;
const agent_column: usize = 1;

fn main() {
    // let mut map = init_map();
    let mut map = TileMap::new(map_width, map_height);
    map.set_obstacle(NodeIndex::new(48), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(49), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(50), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(59), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(60), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(61), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(70), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(71), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(72), TileObstacle::Center);

    let mut a_star = AStar::new(map_width, map_height, 1000);
    let mut paths = vec![];
    let res = a_star.find_path(
        &mut map,
        1000,
        NodeIndex::new(0),
        NodeIndex::new(120),
    );
    println!("r: {:?}", res);
    paths.push(a_star.result(NodeIndex::new(120), map_height));

    let res = a_star.find_path(
        &mut map,
        1000,
        NodeIndex::new(120),
        NodeIndex::new(0),
    );
    println!("r: {:?}", res);
    paths.push(a_star.result(NodeIndex::new(0), map_height));

    let res = a_star.find_path(
        &mut map,
        1000,
        NodeIndex::new(10),
        NodeIndex::new(110),
    );
    println!("r: {:?}", res);
    paths.push(a_star.result(NodeIndex::new(110), map_height));

    let res = a_star.find_path(
        &mut map,
        1000,
        NodeIndex::new(110),
        NodeIndex::new(10),
    );
    println!("r: {:?}", res);
    paths.push(a_star.result(NodeIndex::new(10), map_height));

    // find_path(&mut paths, &mut a_star, &mut map);

    let mut i = 0;
    for path in &mut paths {
        println!("path{}: ", i);
        loop {
            if let Some(pos) = path.next() {
                println!("pos: {:?}", pos);
            }else{
                break;
            }
        }
        i += 1;
    }
}

fn init_map() -> TileMap {
    let mut map = TileMap::new(map_width, map_height);
    map.set_obstacle(NodeIndex::new(48), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(49), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(50), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(59), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(60), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(61), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(70), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(71), TileObstacle::Center);
    map.set_obstacle(NodeIndex::new(72), TileObstacle::Center);

    // for i in 210..290 {
    //     for j in 210..290 {
    //         map.set_obstacle(NodeIndex::new(i * map_height + j), TileObstacle::Center);
    //     }
    // }

    // for i in (260..290).rev() {
    //     for j in 210..240 {
    //         map.set_obstacle(NodeIndex::new(i * map_height + j), TileObstacle::Center);
    //     }
    // }

    // for i in 210..240 {
    //     for j in (260..290).rev() {
    //         map.set_obstacle(NodeIndex::new(i * map_height + j), TileObstacle::Center);
    //     }
    // }

    // for i in (260..290).rev() {
    //     for j in (260..290).rev() {
    //         map.set_obstacle(NodeIndex::new(i * map_height + j), TileObstacle::Center);
    //     }
    // }
    return map;
}

fn find_path(paths: &mut Vec<ResultPath>, a_star: &mut AStar, map: &mut TileMap) {

            let res = a_star.find_path(
                map,
                1000,
                NodeIndex::new(0),
                NodeIndex::new(120),
            );
            println!("r: {:?}", res);
            paths.push(a_star.result(NodeIndex::new(120), map_height));

            let res = a_star.find_path(
                map,
                1000,
                NodeIndex::new(120),
                NodeIndex::new(0),
            );
            println!("r: {:?}", res);
            paths.push(a_star.result(NodeIndex::new(0), map_height));

            let res = a_star.find_path(
                map,
                1000,
                NodeIndex::new(10),
                NodeIndex::new(110),
            );
            println!("r: {:?}", res);
            paths.push(a_star.result(NodeIndex::new(110), map_height));

            let res = a_star.find_path(
                map,
                1000,
                NodeIndex::new(110),
                NodeIndex::new(10),
            );
            println!("r: {:?}", res);
            paths.push(a_star.result(NodeIndex::new(10), map_height));

    // for i in 0..agent_row {
    //     for j in 0..agent_column {
    //         let mut start_x = 150 + i * 10;
    //         let mut start_y = 150 + j * 10;
    //         let mut end_x = 350 - i * 10;
    //         let mut end_y = 350 - j * 10;

    //         let mut res = a_star.find_path(
    //             map,
    //             1000,
    //             NodeIndex::new(start_x * map_height + start_y),
    //             NodeIndex::new(end_x * map_height + end_y),
    //         );
    //         println!("r: {:?}", res);
    //         // paths.push(a_star.result(NodeIndex::new(end_x * map_height + end_y), map_height));

    //         start_x = 350 - i * 10;
    //         start_y = 150 + j * 10;
    //         end_x = 150 + i * 10;
    //         end_y = 350 - j * 10;
    //         res = a_star.find_path(
    //             map,
    //             1000,
    //             NodeIndex::new(start_x * map_height + start_y),
    //             NodeIndex::new(end_x * map_height + end_y),
    //         );
    //         println!("r: {:?}", res);
    //         // paths.push(a_star.result(NodeIndex::new(end_x * map_height + end_y), map_height));

    //         start_x = 150 + i * 10;
    //         start_y = 350 - j * 10;
    //         end_x = 350 - i * 10;
    //         end_x = 150 + j * 10;
    //         res = a_star.find_path(
    //             map,
    //             1000,
    //             NodeIndex::new(start_x * map_height + start_y),
    //             NodeIndex::new(end_x * map_height + end_y),
    //         );
    //         println!("r: {:?}", res);
    //         // paths.push(a_star.result(NodeIndex::new(end_x * map_height + end_y), map_height));

    //         start_x = 350 - i * 10;
    //         start_y = 350 - j * 10;
    //         end_x = 150 + i * 10;
    //         end_x = 150 + j * 10;
    //         res = a_star.find_path(
    //             map,
    //             1000,
    //             NodeIndex::new(start_x * map_height + start_y),
    //             NodeIndex::new(end_x * map_height + end_y),
    //         );
    //         println!("r: {:?}", res);
    //         // paths.push(a_star.result(NodeIndex::new(end_x * map_height + end_y), map_height));
    //     }
    // }
}
