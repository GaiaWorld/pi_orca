/* tslint:disable */
/* eslint-disable */
export const memory: WebAssembly.Memory;
export function __wbg_line_free(a: number): void;
export function __wbg_agent_free(a: number): void;
export function __wbg_get_agent_max_neighbors(a: number): number;
export function __wbg_set_agent_max_neighbors(a: number, b: number): void;
export function __wbg_get_agent_max_speed(a: number): number;
export function __wbg_set_agent_max_speed(a: number, b: number): void;
export function __wbg_get_agent_neighbor_dist(a: number): number;
export function __wbg_set_agent_neighbor_dist(a: number, b: number): void;
export function __wbg_get_agent_new_velocity(a: number): number;
export function __wbg_set_agent_new_velocity(a: number, b: number): void;
export function __wbg_get_agent_position_(a: number): number;
export function __wbg_set_agent_position_(a: number, b: number): void;
export function __wbg_get_agent_pref_velocity(a: number): number;
export function __wbg_set_agent_pref_velocity(a: number, b: number): void;
export function __wbg_get_agent_radius_(a: number): number;
export function __wbg_set_agent_radius_(a: number, b: number): void;
export function __wbg_get_agent_sim_(a: number): number;
export function __wbg_set_agent_sim_(a: number, b: number): void;
export function __wbg_get_agent_time_horizon(a: number): number;
export function __wbg_set_agent_time_horizon(a: number, b: number): void;
export function __wbg_get_agent_time_horizon_obst(a: number): number;
export function __wbg_set_agent_time_horizon_obst(a: number, b: number): void;
export function __wbg_get_agent_velocity_(a: number): number;
export function __wbg_set_agent_velocity_(a: number, b: number): void;
export function __wbg_get_agent_id_(a: number): number;
export function __wbg_set_agent_id_(a: number, b: number): void;
export function __wbg_set_line_point(a: number, b: number): void;
export function __wbg_set_line_direction(a: number, b: number): void;
export function __wbg_get_line_point(a: number): number;
export function __wbg_get_line_direction(a: number): number;
export function __wbg_id_free(a: number): void;
export function __wbg_get_id_0(a: number): number;
export function __wbg_set_id_0(a: number, b: number): void;
export function __wbg_rvosimulator_free(a: number): void;
export function __wbg_get_rvosimulator_global_time(a: number): number;
export function __wbg_set_rvosimulator_global_time(a: number, b: number): void;
export function __wbg_get_rvosimulator_time_step(a: number): number;
export function __wbg_set_rvosimulator_time_step(a: number, b: number): void;
export function rvosimulator_default(): number;
export function rvosimulator_new(a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number): number;
export function rvosimulator_add_agent(a: number, b: number): number;
export function rvosimulator_add_agent2(a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number): number;
export function rvosimulator_add_obstacle(a: number, b: number): number;
export function rvosimulator_do_step(a: number): void;
export function rvosimulator_get_agent_agent_neighbor(a: number, b: number, c: number): number;
export function rvosimulator_get_agent_max_neighbors(a: number, b: number): number;
export function rvosimulator_get_agent_max_speed(a: number, b: number): number;
export function rvosimulator_get_agent_neighbor_dist(a: number, b: number): number;
export function rvosimulator_get_agent_num_agent_neighbors(a: number, b: number): number;
export function rvosimulator_get_agent_num_obstacle_neighbors(a: number, b: number): number;
export function rvosimulator_get_agent_num_orcalines(a: number, b: number): number;
export function rvosimulator_get_agent_obstacle_neighbor(a: number, b: number, c: number): number;
export function rvosimulator_get_agent_orcaline(a: number, b: number, c: number): number;
export function rvosimulator_get_agent_position(a: number, b: number): number;
export function rvosimulator_get_agent_pref_velocity(a: number, b: number): number;
export function rvosimulator_get_agent_radius(a: number, b: number): number;
export function rvosimulator_get_agent_time_horizon(a: number, b: number): number;
export function rvosimulator_get_agent_time_horizon_obst(a: number, b: number): number;
export function rvosimulator_get_agent_velocity(a: number, b: number): number;
export function rvosimulator_get_num_agents(a: number): number;
export function rvosimulator_get_num_obstacle_vertices(a: number): number;
export function rvosimulator_get_obstacle_vertex(a: number, b: number): number;
export function rvosimulator_get_next_obstacle_vertex_no(a: number, b: number): number;
export function rvosimulator_get_prev_obstacle_vertex_no(a: number, b: number): number;
export function rvosimulator_get_agents(a: number, b: number): number;
export function rvosimulator_process_obstacles(): void;
export function rvosimulator_set_agent_defaults(a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number): void;
export function rvosimulator_set_agent_max_neighbors(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_max_speed(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_neighbor_dist(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_position(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_pref_velocity(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_radius(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_time_horizon(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_time_horizon_obst(a: number, b: number, c: number): void;
export function rvosimulator_set_agent_velocity(a: number, b: number, c: number): void;
export function rvosimulator_set_time_step(a: number, b: number): void;
export function rvosimulator_get_global_time(a: number): number;
export function rvosimulator_get_time_step(a: number): number;
export function __wbg_vector2_free(a: number): void;
export function __wbg_get_vector2_x(a: number): number;
export function __wbg_set_vector2_x(a: number, b: number): void;
export function __wbg_get_vector2_y(a: number): number;
export function __wbg_set_vector2_y(a: number, b: number): void;
export function vector2_default(): number;
export function vector2_add(a: number, b: number): number;
export function vector2_sub(a: number, b: number): number;
export function vector2_mul(a: number, b: number): number;
export function vector2_mul_number(a: number, b: number): number;
export function vector2_div(a: number, b: number): number;
export function vector2_neg(a: number): number;
export function vector2_abs(a: number): number;
export function vector2_abs_sq(a: number): number;
export function vector2_det(a: number, b: number): number;
export function vector2_normalize(a: number): number;
export function vector2_dist_sq_point_line_segment(a: number, b: number, c: number): number;
export function vector2_left_of(a: number, b: number, c: number): number;
export function vector2_x(a: number): number;
export function vector2_y(a: number): number;
export function vector2_new(a: number, b: number): number;
export function vector2_sqr(a: number): number;
export function __wbg_nodeindex_free(a: number): void;
export function nodeindex_new(a: number): number;
export function nodeindex_index(a: number): number;
export function __wbg_tilemap_free(a: number): void;
export function __wbg_resultpath_free(a: number): void;
export function resultpath_next(a: number): number;
export function tilemap_new(a: number, b: number): number;
export function tilemap_set_obstacle(a: number, b: number, c: number): void;
export function __wbg_astar_free(a: number): void;
export function astar_new(a: number, b: number, c: number): number;
export function astar_find_path(a: number, b: number, c: number, d: number, e: number): number;
export function astar_result(a: number, b: number, c: number): number;
export function __wbg_obstacle_free(a: number): void;
export function __wbg_get_obstacle_is_convex(a: number): number;
export function __wbg_set_obstacle_is_convex(a: number, b: number): void;
export function __wbg_get_obstacle_next_obstacle(a: number): number;
export function __wbg_set_obstacle_next_obstacle(a: number, b: number): void;
export function __wbg_get_obstacle_point_(a: number): number;
export function __wbg_set_obstacle_point_(a: number, b: number): void;
export function __wbg_get_obstacle_prev_obstacle(a: number): number;
export function __wbg_set_obstacle_prev_obstacle(a: number, b: number): void;
export function __wbg_get_obstacle_unit_dir(a: number): number;
export function __wbg_set_obstacle_unit_dir(a: number, b: number): void;
export function __wbg_get_obstacle_id_(a: number): number;
export function __wbg_set_obstacle_id_(a: number, b: number): void;
export function obstacle_default(): number;
export function __wbg_vertices_free(a: number): void;
export function vertices_new(): number;
export function vertices_add(a: number, b: number): void;
export function vertices_get(a: number, b: number): number;
export function vertices_len(a: number): number;
