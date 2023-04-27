/* tslint:disable */
/* eslint-disable */
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
export class RVOSimulator {
  free(): void;
/**
* @param {number} _max_obstacle
* @returns {RVOSimulator}
*/
  static default(_max_obstacle: number): RVOSimulator;
/**
* @param {number} _max_obstacle
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
  static new(_max_obstacle: number, time_step: number, neighbor_dist: number, max_neighbors: number, time_horizon: number, time_horizon_obst: number, radius: number, max_speed: number, velocity: Vector2): RVOSimulator;
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
* @param {number} id
* @returns {boolean}
*/
  remove_agent(id: number): boolean;
/**
*
*     * @brief 为模拟添加新障碍。
*     * @param[in] vertices 逆时针顺序排列的多边形障碍物的顶点列表。
*     * @return 障碍物第一个顶点的编号，当顶点数小于2时为返回usize::MAX。
*     * @note 要添加“负面”障碍，例如环境周围的边界多边形，顶点应按顺时针顺序列出。
*     
* @param {Vertices} vertices
* @returns {number}
*/
  add_obstacle(vertices: Vertices): number;
/**
* @param {number} id
* @returns {boolean}
*/
  remove_obstacle(id: number): boolean;
/**
*/
  do_step(): void;
/**
*
*    * @brief     为添加的任何新代理设置默认属性。
*    * @param[in] neighborDist    新代理在导航中考虑的默认最大中心点到中心点到其他代理的最大距离。
*                                 这个数字越大，模拟的运行时间就越长。
*                                 如果数字太低，模拟将不安全。
*                                 必须是非负数。
*    * @param[in] maxNeighbors    新代理在导航中考虑的默认最大其他代理数。
*                                 这个数字越大，模拟的运行时间越长。
*                                 如果数字太低，模拟将不安全。
*    * @param[in] timeHorizon     模拟计算的新代理速度相对于其他代理安全的默认最小时间量。
*                                 这个数字越大，代理就会越快响应其他代理的存在，但代理在选择速度方面的自由度就越小。
*                                 必须是正数。
*    * @param[in] timeHorizonObst 通过模拟计算的新代理的速度相对于障碍物是安全的默认最小时间量。
*                                 这个数字越大，代理越快响应障碍的存在，但代理的自由度越低 选择它的速度。
*                                 必须是正数。
*    * @param[in] radius          新代理的默认半径。 必须是非负数。
*    * @param[in] maxSpeed        新代理的默认最大速度。 必须是非负数。
*    
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
* @param {number} neighbor_no
* @returns {number | undefined}
*/
  get_agent_agent_neighbor(agent_no: number, neighbor_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_max_neighbors(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_max_speed(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_neighbor_dist(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_num_agent_neighbors(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_num_obstacle_neighbors(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_num_orcalines(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @param {number} neighbor_no
* @returns {number | undefined}
*/
  get_agent_obstacle_neighbor(agent_no: number, neighbor_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {Vector2 | undefined}
*/
  get_agent_position(agent_no: number): Vector2 | undefined;
/**
* @param {number} agent_no
* @returns {Vector2 | undefined}
*/
  get_agent_pref_velocity(agent_no: number): Vector2 | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_radius(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_time_horizon(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {number | undefined}
*/
  get_agent_time_horizon_obst(agent_no: number): number | undefined;
/**
* @param {number} agent_no
* @returns {Vector2 | undefined}
*/
  get_agent_velocity(agent_no: number): Vector2 | undefined;
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
* @returns {Vector2 | undefined}
*/
  get_obstacle_vertex(vertex_no: number): Vector2 | undefined;
/**
* @param {number} vertex_no
* @returns {number | undefined}
*/
  get_next_obstacle_vertex_no(vertex_no: number): number | undefined;
/**
* @param {number} vertex_no
* @returns {number | undefined}
*/
  get_prev_obstacle_vertex_no(vertex_no: number): number | undefined;
/**
* @returns {number}
*/
  get_time_step(): number;
/**
* @param {number} agent_no
* @param {number} max_neighbors
* @returns {boolean}
*/
  set_agent_max_neighbors(agent_no: number, max_neighbors: number): boolean;
/**
* @param {number} agent_no
* @param {number} max_speed
* @returns {boolean}
*/
  set_agent_max_speed(agent_no: number, max_speed: number): boolean;
/**
* @param {number} agent_no
* @param {number} neighbor_dist
* @returns {boolean}
*/
  set_agent_neighbor_dist(agent_no: number, neighbor_dist: number): boolean;
/**
* @param {number} agent_no
* @param {Vector2} position
* @returns {boolean}
*/
  set_agent_position(agent_no: number, position: Vector2): boolean;
/**
* @param {number} agent_no
* @param {Vector2} pref_velocity
* @returns {boolean}
*/
  set_agent_pref_velocity(agent_no: number, pref_velocity: Vector2): boolean;
/**
* @param {number} agent_no
* @param {number} radius
* @returns {boolean}
*/
  set_agent_radius(agent_no: number, radius: number): boolean;
/**
* @param {number} agent_no
* @param {number} time_horizon
* @returns {boolean}
*/
  set_agent_time_horizon(agent_no: number, time_horizon: number): boolean;
/**
* @param {number} agent_no
* @param {number} time_horizon_obst
* @returns {boolean}
*/
  set_agent_time_horizon_obst(agent_no: number, time_horizon_obst: number): boolean;
/**
* @param {number} agent_no
* @param {Vector2} velocity
* @returns {boolean}
*/
  set_agent_velocity(agent_no: number, velocity: Vector2): boolean;
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
  readonly __wbg_rvosimulator_free: (a: number) => void;
  readonly __wbg_get_rvosimulator_global_time: (a: number) => number;
  readonly __wbg_set_rvosimulator_global_time: (a: number, b: number) => void;
  readonly __wbg_get_rvosimulator_time_step: (a: number) => number;
  readonly __wbg_set_rvosimulator_time_step: (a: number, b: number) => void;
  readonly rvosimulator_default: (a: number) => number;
  readonly rvosimulator_new: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
  readonly rvosimulator_add_agent: (a: number, b: number) => number;
  readonly rvosimulator_add_agent2: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
  readonly rvosimulator_remove_agent: (a: number, b: number) => number;
  readonly rvosimulator_add_obstacle: (a: number, b: number) => number;
  readonly rvosimulator_remove_obstacle: (a: number, b: number) => number;
  readonly rvosimulator_do_step: (a: number) => void;
  readonly rvosimulator_set_agent_defaults: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => void;
  readonly rvosimulator_get_agent_agent_neighbor: (a: number, b: number, c: number, d: number) => void;
  readonly rvosimulator_get_agent_max_neighbors: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_max_speed: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_neighbor_dist: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_num_agent_neighbors: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_num_obstacle_neighbors: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_num_orcalines: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_obstacle_neighbor: (a: number, b: number, c: number, d: number) => void;
  readonly rvosimulator_get_agent_position: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_pref_velocity: (a: number, b: number) => number;
  readonly rvosimulator_get_agent_radius: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_time_horizon: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_time_horizon_obst: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_agent_velocity: (a: number, b: number) => number;
  readonly rvosimulator_get_global_time: (a: number) => number;
  readonly rvosimulator_get_num_agents: (a: number) => number;
  readonly rvosimulator_get_num_obstacle_vertices: (a: number) => number;
  readonly rvosimulator_get_obstacle_vertex: (a: number, b: number) => number;
  readonly rvosimulator_get_next_obstacle_vertex_no: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_prev_obstacle_vertex_no: (a: number, b: number, c: number) => void;
  readonly rvosimulator_get_time_step: (a: number) => number;
  readonly rvosimulator_set_agent_max_neighbors: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_max_speed: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_neighbor_dist: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_position: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_pref_velocity: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_radius: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_time_horizon: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_time_horizon_obst: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_agent_velocity: (a: number, b: number, c: number) => number;
  readonly rvosimulator_set_time_step: (a: number, b: number) => void;
  readonly __wbg_vertices_free: (a: number) => void;
  readonly vertices_new: () => number;
  readonly vertices_add: (a: number, b: number) => void;
  readonly vertices_get: (a: number, b: number) => number;
  readonly vertices_len: (a: number) => number;
  readonly __wbg_id_free: (a: number) => void;
  readonly __wbg_get_id_0: (a: number) => number;
  readonly __wbg_set_id_0: (a: number, b: number) => void;
  readonly __wbg_vector2_free: (a: number) => void;
  readonly __wbg_get_vector2_x: (a: number) => number;
  readonly __wbg_set_vector2_x: (a: number, b: number) => void;
  readonly __wbg_get_vector2_y: (a: number) => number;
  readonly __wbg_set_vector2_y: (a: number, b: number) => void;
  readonly vector2_default: () => number;
  readonly vector2_new: (a: number, b: number) => number;
  readonly vector2_x: (a: number) => number;
  readonly vector2_y: (a: number) => number;
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
  readonly vector2_sqr: (a: number) => number;
  readonly __wbindgen_add_to_stack_pointer: (a: number) => number;
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
