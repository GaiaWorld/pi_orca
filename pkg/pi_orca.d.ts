/* tslint:disable */
/* eslint-disable */
/**
*/
export enum TileObstacle {
  Right = 1,
  Down = 2,
  Center = 4,
}
/**
*/
export class AStar {
  free(): void;
/**
* @param {number} row
* @param {number} column
* @param {number} node_number
* @returns {AStar}
*/
  static new(row: number, column: number, node_number: number): AStar;
/**
* @param {TileMap} tile_map
* @param {number} max_number
* @param {NodeIndex} start
* @param {NodeIndex} end
* @returns {NodeIndex | undefined}
*/
  find_path(tile_map: TileMap, max_number: number, start: NodeIndex, end: NodeIndex): NodeIndex | undefined;
/**
* @param {NodeIndex} node
* @param {number} column
* @returns {ResultPath}
*/
  result(node: NodeIndex, column: number): ResultPath;
}
/**
*/
export class Agent {
  free(): void;
/**
*/
  id_: ID;
/**
*/
  max_neighbors: number;
/**
*/
  max_speed: number;
/**
*/
  neighbor_dist: number;
/**
*/
  new_velocity: Vector2;
/**
*/
  position_: Vector2;
/**
*/
  pref_velocity: Vector2;
/**
*/
  radius_: number;
/**
*/
  sim_: number;
/**
*/
  time_horizon: number;
/**
*/
  time_horizon_obst: number;
/**
*/
  velocity_: Vector2;
}
/**
*/
export class ID {
  free(): void;
/**
*/
  0: number;
}
/**
*/
export class Line {
  free(): void;
/**
*
*     * \brief     The direction of the directed line.
*     
*/
  direction: Vector2;
/**
*
*     * \brief     A point on the directed line.
*     
*/
  point: Vector2;
}
/**
*/
export class NodeIndex {
  free(): void;
/**
* @param {number} index
* @returns {NodeIndex}
*/
  static new(index: number): NodeIndex;
/**
* @returns {number}
*/
  index(): number;
}
/**
*/
export class Obstacle {
  free(): void;
/**
* @returns {Obstacle}
*/
  static default(): Obstacle;
/**
*/
  id_: number;
/**
*/
  is_convex: boolean;
/**
*/
  next_obstacle: number;
/**
*/
  point_: Vector2;
/**
*/
  prev_obstacle: number;
/**
*/
  unit_dir: Vector2;
}
/**
*/
export class RVOSimulator {
  free(): void;
/**
* @returns {RVOSimulator}
*/
  static default(): RVOSimulator;
/**
* @param {number} max_obstacle
* @param {number} time_step
* @param {number} neighbor_dist
* @param {number} max_neighbors
* @param {number} time_horizon
* @param {number} time_horizon_obst
* @param {number} radius
* @param {number} max_speed
* @param {Vector2} velocity
* @returns {RVOSimulator}
*/
  static new(max_obstacle: number, time_step: number, neighbor_dist: number, max_neighbors: number, time_horizon: number, time_horizon_obst: number, radius: number, max_speed: number, velocity: Vector2): RVOSimulator;
/**
* @param {Vector2} position
* @returns {number}
*/
  add_agent(position: Vector2): number;
/**
* @param {Vector2} position
* @param {number} neighbor_dist
* @param {number} max_neighbors
* @param {number} time_horizon
* @param {number} time_horizon_obst
* @param {number} radius
* @param {number} max_speed
* @param {Vector2} velocity
* @returns {number}
*/
  add_agent2(position: Vector2, neighbor_dist: number, max_neighbors: number, time_horizon: number, time_horizon_obst: number, radius: number, max_speed: number, velocity: Vector2): number;
/**
* @param {Vertices} vertices
* @returns {number}
*/
  add_obstacle(vertices: Vertices): number;
/**
*/
  do_step(): void;
/**
* @param {number} agent_no
* @param {number} neighbor_no
* @returns {number}
*/
  get_agent_agent_neighbor(agent_no: number, neighbor_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_max_neighbors(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_max_speed(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_neighbor_dist(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_num_agent_neighbors(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_num_obstacle_neighbors(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_num_orcalines(agent_no: number): number;
/**
* @param {number} agent_no
* @param {number} neighbor_no
* @returns {number}
*/
  get_agent_obstacle_neighbor(agent_no: number, neighbor_no: number): number;
/**
* @param {number} agent_no
* @param {number} line_no
* @returns {Line}
*/
  get_agent_orcaline(agent_no: number, line_no: number): Line;
/**
* @param {number} agent_no
* @returns {Vector2}
*/
  get_agent_position(agent_no: number): Vector2;
/**
* @param {number} agent_no
* @returns {Vector2}
*/
  get_agent_pref_velocity(agent_no: number): Vector2;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_radius(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_time_horizon(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agent_time_horizon_obst(agent_no: number): number;
/**
* @param {number} agent_no
* @returns {Vector2}
*/
  get_agent_velocity(agent_no: number): Vector2;
/**
* @returns {number}
*/
  get_global_time(): number;
/**
* @returns {number}
*/
  get_num_agents(): number;
/**
* @returns {number}
*/
  get_num_obstacle_vertices(): number;
/**
* @param {number} vertex_no
* @returns {Vector2}
*/
  get_obstacle_vertex(vertex_no: number): Vector2;
/**
* @param {number} vertex_no
* @returns {number}
*/
  get_next_obstacle_vertex_no(vertex_no: number): number;
/**
* @param {number} vertex_no
* @returns {number}
*/
  get_prev_obstacle_vertex_no(vertex_no: number): number;
/**
* @returns {number}
*/
  get_time_step(): number;
/**
* @param {number} agent_no
* @returns {number}
*/
  get_agents(agent_no: number): number;
/**
*/
  static process_obstacles(): void;
/**
* @param {number} neighbor_dist
* @param {number} max_neighbors
* @param {number} time_horizon
* @param {number} time_horizon_obst
* @param {number} radius
* @param {number} max_speed
* @param {Vector2} velocity
*/
  set_agent_defaults(neighbor_dist: number, max_neighbors: number, time_horizon: number, time_horizon_obst: number, radius: number, max_speed: number, velocity: Vector2): void;
/**
* @param {number} agent_no
* @param {number} max_neighbors
*/
  set_agent_max_neighbors(agent_no: number, max_neighbors: number): void;
/**
* @param {number} agent_no
* @param {number} max_speed
*/
  set_agent_max_speed(agent_no: number, max_speed: number): void;
/**
* @param {number} agent_no
* @param {number} neighbor_dist
*/
  set_agent_neighbor_dist(agent_no: number, neighbor_dist: number): void;
/**
* @param {number} agent_no
* @param {Vector2} position
*/
  set_agent_position(agent_no: number, position: Vector2): void;
/**
* @param {number} agent_no
* @param {Vector2} pref_velocity
*/
  set_agent_pref_velocity(agent_no: number, pref_velocity: Vector2): void;
/**
* @param {number} agent_no
* @param {number} radius
*/
  set_agent_radius(agent_no: number, radius: number): void;
/**
* @param {number} agent_no
* @param {number} time_horizon
*/
  set_agent_time_horizon(agent_no: number, time_horizon: number): void;
/**
* @param {number} agent_no
* @param {number} time_horizon_obst
*/
  set_agent_time_horizon_obst(agent_no: number, time_horizon_obst: number): void;
/**
* @param {number} agent_no
* @param {Vector2} velocity
*/
  set_agent_velocity(agent_no: number, velocity: Vector2): void;
/**
* @param {number} time_step
*/
  set_time_step(time_step: number): void;
/**
*/
  global_time: number;
/**
*/
  time_step: number;
}
/**
*/
export class ResultPath {
  free(): void;
/**
* @returns {Vector2 | undefined}
*/
  next(): Vector2 | undefined;
}
/**
*/
export class TileMap {
  free(): void;
/**
* @param {number} row
* @param {number} column
* @returns {TileMap}
*/
  static new(row: number, column: number): TileMap;
/**
* @param {NodeIndex} index
* @param {number} obstacle
*/
  set_obstacle(index: NodeIndex, obstacle: number): void;
}
/**
*/
export class Vector2 {
  free(): void;
/**
* @returns {Vector2}
*/
  static default(): Vector2;
/**
* @param {number} x
* @param {number} y
* @returns {Vector2}
*/
  static new(x: number, y: number): Vector2;
/**
* @returns {number}
*/
  x(): number;
/**
* @returns {number}
*/
  y(): number;
/**
* @param {Vector2} other
* @returns {Vector2}
*/
  add(other: Vector2): Vector2;
/**
* @param {Vector2} other
* @returns {Vector2}
*/
  sub(other: Vector2): Vector2;
/**
* @param {Vector2} other
* @returns {number}
*/
  mul(other: Vector2): number;
/**
* @param {number} other
* @returns {Vector2}
*/
  mul_number(other: number): Vector2;
/**
* @param {number} other
* @returns {Vector2}
*/
  div(other: number): Vector2;
/**
* @returns {Vector2}
*/
  neg(): Vector2;
/**
* @param {Vector2} v
* @returns {number}
*/
  static abs(v: Vector2): number;
/**
* @param {Vector2} v
* @returns {number}
*/
  static abs_sq(v: Vector2): number;
/**
* @param {Vector2} v1
* @param {Vector2} v2
* @returns {number}
*/
  static det(v1: Vector2, v2: Vector2): number;
/**
* @param {Vector2} vector
* @returns {Vector2}
*/
  static normalize(vector: Vector2): Vector2;
/**
*
*     * \brief      Computes the squared distance from a line segment with the
*     *             specified endpoints to a specified point.
*     * \param      a               The first endpoint of the line segment.
*     * \param      b               The second endpoint of the line segment.
*     * \param      c               The point to which the squared distance is to
*     *                             be calculated.
*     * \return     The squared distance from the line segment to the point.
*     
* @param {Vector2} a
* @param {Vector2} b
* @param {Vector2} c
* @returns {number}
*/
  static dist_sq_point_line_segment(a: Vector2, b: Vector2, c: Vector2): number;
/**
*
*     * \brief      Computes the signed distance from a line connecting the
*     *             specified points to a specified point.
*     * \param      a               The first point on the line.
*     * \param      b               The second point on the line.
*     * \param      c               The point to which the signed distance is to
*     *                             be calculated.
*     * \return     Positive when the point c lies to the left of the line ab.
*     
* @param {Vector2} a
* @param {Vector2} b
* @param {Vector2} c
* @returns {number}
*/
  static left_of(a: Vector2, b: Vector2, c: Vector2): number;
/**
*
*     * \brief      Computes the square of a float.
*     * \param      a               The float to be squared.
*     * \return     The square of the float.
*     
* @param {number} a
* @returns {number}
*/
  static sqr(a: number): number;
/**
*/
  x: number;
/**
*/
  y: number;
}
/**
*/
export class Vertices {
  free(): void;
/**
* @returns {Vertices}
*/
  static new(): Vertices;
/**
* @param {Vector2} vertex
*/
  add(vertex: Vector2): void;
/**
* @param {number} index
* @returns {Vector2}
*/
  get(index: number): Vector2;
/**
* @returns {number}
*/
  len(): number;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
  readonly memory: WebAssembly.Memory;
  readonly __wbg_line_free: (a: number) => void;
  readonly __wbg_agent_free: (a: number) => void;
  readonly __wbg_get_agent_max_neighbors: (a: number) => number;
  readonly __wbg_set_agent_max_neighbors: (a: number, b: number) => void;
  readonly __wbg_get_agent_max_speed: (a: number) => number;
  readonly __wbg_set_agent_max_speed: (a: number, b: number) => void;
  readonly __wbg_get_agent_neighbor_dist: (a: number) => number;
  readonly __wbg_set_agent_neighbor_dist: (a: number, b: number) => void;
  readonly __wbg_get_agent_new_velocity: (a: number) => number;
  readonly __wbg_set_agent_new_velocity: (a: number, b: number) => void;
  readonly __wbg_get_agent_position_: (a: number) => number;
  readonly __wbg_set_agent_position_: (a: number, b: number) => void;
  readonly __wbg_get_agent_pref_velocity: (a: number) => number;
  readonly __wbg_set_agent_pref_velocity: (a: number, b: number) => void;
  readonly __wbg_get_agent_radius_: (a: number) => number;
  readonly __wbg_set_agent_radius_: (a: number, b: number) => void;
  readonly __wbg_get_agent_sim_: (a: number) => number;
  readonly __wbg_set_agent_sim_: (a: number, b: number) => void;
  readonly __wbg_get_agent_time_horizon: (a: number) => number;
  readonly __wbg_set_agent_time_horizon: (a: number, b: number) => void;
  readonly __wbg_get_agent_time_horizon_obst: (a: number) => number;
  readonly __wbg_set_agent_time_horizon_obst: (a: number, b: number) => void;
  readonly __wbg_get_agent_velocity_: (a: number) => number;
  readonly __wbg_set_agent_velocity_: (a: number, b: number) => void;
  readonly __wbg_get_agent_id_: (a: number) => number;
  readonly __wbg_set_agent_id_: (a: number, b: number) => void;
  readonly __wbg_set_line_point: (a: number, b: number) => void;
  readonly __wbg_set_line_direction: (a: number, b: number) => void;
  readonly __wbg_get_line_point: (a: number) => number;
  readonly __wbg_get_line_direction: (a: number) => number;
  readonly __wbg_id_free: (a: number) => void;
  readonly __wbg_get_id_0: (a: number) => number;
  readonly __wbg_set_id_0: (a: number, b: number) => void;
  readonly __wbg_rvosimulator_free: (a: number) => void;
  readonly __wbg_get_rvosimulator_global_time: (a: number) => number;
  readonly __wbg_set_rvosimulator_global_time: (a: number, b: number) => void;
  readonly __wbg_get_rvosimulator_time_step: (a: number) => number;
  readonly __wbg_set_rvosimulator_time_step: (a: number, b: number) => void;
  readonly rvosimulator_default: () => number;
  readonly rvosimulator_new: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
  readonly rvosimulator_add_agent: (a: number, b: number) => number;
  readonly rvosimulator_add_agent2: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
  readonly rvosimulator_add_obstacle: (a: number, b: number) => number;
  readonly rvosimulator_do_step: (a: number) => void;
  readonly rvosimulator_get_agent_agent_neighbor: (a: number, b: number, c: number) => number;
  readonly rvosimulator_get_agent_max_neighbors: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_max_speed: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_neighbor_dist: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_num_agent_neighbors: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_num_obstacle_neighbors: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_num_orcalines: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_obstacle_neighbor: (a: number, b: number, c: number) => number;
  readonly rvosimulator_get_agent_orcaline: (a: number, b: number, c: number) => number;
  readonly rvosimulator_get_agent_position: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_pref_velocity: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_radius: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_time_horizon: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_time_horizon_obst: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_velocity: (a: number, b: number) => number;
  readonly rvosimulator_get_num_agents: (a: number) => number;
  readonly rvosimulator_get_num_obstacle_vertices: (a: number) => number;
  readonly rvosimulator_get_obstacle_vertex: (a: number, b: number) => number;
  readonly rvosimulator_get_next_obstacle_vertex_no: (a: number, b: number) => number;
  readonly rvosimulator_get_prev_obstacle_vertex_no: (a: number, b: number) => number;
  readonly rvosimulator_get_agents: (a: number, b: number) => number;
  readonly rvosimulator_process_obstacles: () => void;
  readonly rvosimulator_set_agent_defaults: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => void;
  readonly rvosimulator_set_agent_max_neighbors: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_max_speed: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_neighbor_dist: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_position: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_pref_velocity: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_radius: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_time_horizon: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_time_horizon_obst: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_agent_velocity: (a: number, b: number, c: number) => void;
  readonly rvosimulator_set_time_step: (a: number, b: number) => void;
  readonly rvosimulator_get_global_time: (a: number) => number;
  readonly rvosimulator_get_time_step: (a: number) => number;
  readonly __wbg_vector2_free: (a: number) => void;
  readonly __wbg_get_vector2_x: (a: number) => number;
  readonly __wbg_set_vector2_x: (a: number, b: number) => void;
  readonly __wbg_get_vector2_y: (a: number) => number;
  readonly __wbg_set_vector2_y: (a: number, b: number) => void;
  readonly vector2_default: () => number;
  readonly vector2_add: (a: number, b: number) => number;
  readonly vector2_sub: (a: number, b: number) => number;
  readonly vector2_mul: (a: number, b: number) => number;
  readonly vector2_mul_number: (a: number, b: number) => number;
  readonly vector2_div: (a: number, b: number) => number;
  readonly vector2_neg: (a: number) => number;
  readonly vector2_abs: (a: number) => number;
  readonly vector2_abs_sq: (a: number) => number;
  readonly vector2_det: (a: number, b: number) => number;
  readonly vector2_normalize: (a: number) => number;
  readonly vector2_dist_sq_point_line_segment: (a: number, b: number, c: number) => number;
  readonly vector2_left_of: (a: number, b: number, c: number) => number;
  readonly vector2_x: (a: number) => number;
  readonly vector2_y: (a: number) => number;
  readonly vector2_new: (a: number, b: number) => number;
  readonly vector2_sqr: (a: number) => number;
  readonly __wbg_nodeindex_free: (a: number) => void;
  readonly nodeindex_new: (a: number) => number;
  readonly nodeindex_index: (a: number) => number;
  readonly __wbg_tilemap_free: (a: number) => void;
  readonly __wbg_resultpath_free: (a: number) => void;
  readonly resultpath_next: (a: number) => number;
  readonly tilemap_new: (a: number, b: number) => number;
  readonly tilemap_set_obstacle: (a: number, b: number, c: number) => void;
  readonly __wbg_astar_free: (a: number) => void;
  readonly astar_new: (a: number, b: number, c: number) => number;
  readonly astar_find_path: (a: number, b: number, c: number, d: number, e: number) => number;
  readonly astar_result: (a: number, b: number, c: number) => number;
  readonly __wbg_obstacle_free: (a: number) => void;
  readonly __wbg_get_obstacle_is_convex: (a: number) => number;
  readonly __wbg_set_obstacle_is_convex: (a: number, b: number) => void;
  readonly __wbg_get_obstacle_next_obstacle: (a: number) => number;
  readonly __wbg_set_obstacle_next_obstacle: (a: number, b: number) => void;
  readonly __wbg_get_obstacle_point_: (a: number) => number;
  readonly __wbg_set_obstacle_point_: (a: number, b: number) => void;
  readonly __wbg_get_obstacle_prev_obstacle: (a: number) => number;
  readonly __wbg_set_obstacle_prev_obstacle: (a: number, b: number) => void;
  readonly __wbg_get_obstacle_unit_dir: (a: number) => number;
  readonly __wbg_set_obstacle_unit_dir: (a: number, b: number) => void;
  readonly __wbg_get_obstacle_id_: (a: number) => number;
  readonly __wbg_set_obstacle_id_: (a: number, b: number) => void;
  readonly obstacle_default: () => number;
  readonly __wbg_vertices_free: (a: number) => void;
  readonly vertices_new: () => number;
  readonly vertices_add: (a: number, b: number) => void;
  readonly vertices_get: (a: number, b: number) => number;
  readonly vertices_len: (a: number) => number;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;
/**
* Instantiates the given `module`, which can either be bytes or
* a precompiled `WebAssembly.Module`.
*
* @param {SyncInitInput} module
*
* @returns {InitOutput}
*/
export function initSync(module: SyncInitInput): InitOutput;

/**
* If `module_or_path` is {RequestInfo} or {URL}, makes a request and
* for everything else, calls `WebAssembly.instantiate` directly.
*
* @param {InitInput | Promise<InitInput>} module_or_path
*
* @returns {Promise<InitOutput>}
*/
export default function init (module_or_path?: InitInput | Promise<InitInput>): Promise<InitOutput>;
