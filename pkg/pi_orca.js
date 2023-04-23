let wasm;

const cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });

cachedTextDecoder.decode();

let cachedUint8Memory0 = null;

function getUint8Memory0() {
    if (cachedUint8Memory0 === null || cachedUint8Memory0.byteLength === 0) {
        cachedUint8Memory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8Memory0;
}

function getStringFromWasm0(ptr, len) {
    return cachedTextDecoder.decode(getUint8Memory0().subarray(ptr, ptr + len));
}

function _assertClass(instance, klass) {
    if (!(instance instanceof klass)) {
        throw new Error(`expected instance of ${klass.name}`);
    }
    return instance.ptr;
}
/**
*/
export const TileObstacle = Object.freeze({ Right:1,"1":"Right",Down:2,"2":"Down",Center:4,"4":"Center", });
/**
*/
export class AStar {

    static __wrap(ptr) {
        const obj = Object.create(AStar.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_astar_free(ptr);
    }
    /**
    * @param {number} row
    * @param {number} column
    * @param {number} node_number
    * @returns {AStar}
    */
    static new(row, column, node_number) {
        const ret = wasm.astar_new(row, column, node_number);
        return AStar.__wrap(ret);
    }
    /**
    * @param {TileMap} tile_map
    * @param {number} max_number
    * @param {NodeIndex} start
    * @param {NodeIndex} end
    * @returns {NodeIndex | undefined}
    */
    find_path(tile_map, max_number, start, end) {
        _assertClass(tile_map, TileMap);
        _assertClass(start, NodeIndex);
        var ptr0 = start.__destroy_into_raw();
        _assertClass(end, NodeIndex);
        var ptr1 = end.__destroy_into_raw();
        const ret = wasm.astar_find_path(this.ptr, tile_map.ptr, max_number, ptr0, ptr1);
        return ret === 0 ? undefined : NodeIndex.__wrap(ret);
    }
    /**
    * @param {NodeIndex} node
    * @param {number} column
    * @returns {ResultPath}
    */
    result(node, column) {
        _assertClass(node, NodeIndex);
        var ptr0 = node.__destroy_into_raw();
        const ret = wasm.astar_result(this.ptr, ptr0, column);
        return ResultPath.__wrap(ret);
    }
}
/**
*/
export class Agent {

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_agent_free(ptr);
    }
    /**
    * @returns {number}
    */
    get max_neighbors() {
        const ret = wasm.__wbg_get_agent_max_neighbors(this.ptr);
        return ret >>> 0;
    }
    /**
    * @param {number} arg0
    */
    set max_neighbors(arg0) {
        wasm.__wbg_set_agent_max_neighbors(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get max_speed() {
        const ret = wasm.__wbg_get_agent_max_speed(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set max_speed(arg0) {
        wasm.__wbg_set_agent_max_speed(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get neighbor_dist() {
        const ret = wasm.__wbg_get_agent_neighbor_dist(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set neighbor_dist(arg0) {
        wasm.__wbg_set_agent_neighbor_dist(this.ptr, arg0);
    }
    /**
    * @returns {Vector2}
    */
    get new_velocity() {
        const ret = wasm.__wbg_get_agent_new_velocity(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} arg0
    */
    set new_velocity(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_agent_new_velocity(this.ptr, ptr0);
    }
    /**
    * @returns {Vector2}
    */
    get position_() {
        const ret = wasm.__wbg_get_agent_position_(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} arg0
    */
    set position_(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_agent_position_(this.ptr, ptr0);
    }
    /**
    * @returns {Vector2}
    */
    get pref_velocity() {
        const ret = wasm.__wbg_get_agent_pref_velocity(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} arg0
    */
    set pref_velocity(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_agent_pref_velocity(this.ptr, ptr0);
    }
    /**
    * @returns {number}
    */
    get radius_() {
        const ret = wasm.__wbg_get_agent_radius_(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set radius_(arg0) {
        wasm.__wbg_set_agent_radius_(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get sim_() {
        const ret = wasm.__wbg_get_agent_sim_(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set sim_(arg0) {
        wasm.__wbg_set_agent_sim_(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get time_horizon() {
        const ret = wasm.__wbg_get_agent_time_horizon(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set time_horizon(arg0) {
        wasm.__wbg_set_agent_time_horizon(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get time_horizon_obst() {
        const ret = wasm.__wbg_get_agent_time_horizon_obst(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set time_horizon_obst(arg0) {
        wasm.__wbg_set_agent_time_horizon_obst(this.ptr, arg0);
    }
    /**
    * @returns {Vector2}
    */
    get velocity_() {
        const ret = wasm.__wbg_get_agent_velocity_(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} arg0
    */
    set velocity_(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_agent_velocity_(this.ptr, ptr0);
    }
    /**
    * @returns {ID}
    */
    get id_() {
        const ret = wasm.__wbg_get_agent_id_(this.ptr);
        return ID.__wrap(ret);
    }
    /**
    * @param {ID} arg0
    */
    set id_(arg0) {
        _assertClass(arg0, ID);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_agent_id_(this.ptr, ptr0);
    }
}
/**
*/
export class ID {

    static __wrap(ptr) {
        const obj = Object.create(ID.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_id_free(ptr);
    }
    /**
    * @returns {number}
    */
    get 0() {
        const ret = wasm.__wbg_get_id_0(this.ptr);
        return ret >>> 0;
    }
    /**
    * @param {number} arg0
    */
    set 0(arg0) {
        wasm.__wbg_set_id_0(this.ptr, arg0);
    }
}
/**
*/
export class Line {

    static __wrap(ptr) {
        const obj = Object.create(Line.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_line_free(ptr);
    }
    /**
    *
    *     * \brief     A point on the directed line.
    *
    * @returns {Vector2}
    */
    get point() {
        const ret = wasm.__wbg_get_agent_new_velocity(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    *
    *     * \brief     A point on the directed line.
    *
    * @param {Vector2} arg0
    */
    set point(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_agent_new_velocity(this.ptr, ptr0);
    }
    /**
    *
    *     * \brief     The direction of the directed line.
    *
    * @returns {Vector2}
    */
    get direction() {
        const ret = wasm.__wbg_get_agent_position_(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    *
    *     * \brief     The direction of the directed line.
    *
    * @param {Vector2} arg0
    */
    set direction(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_agent_position_(this.ptr, ptr0);
    }
}
/**
*/
export class NodeIndex {

    static __wrap(ptr) {
        const obj = Object.create(NodeIndex.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_nodeindex_free(ptr);
    }
    /**
    * @param {number} index
    * @returns {NodeIndex}
    */
    static new(index) {
        const ret = wasm.nodeindex_new(index);
        return NodeIndex.__wrap(ret);
    }
    /**
    * @returns {number}
    */
    index() {
        const ret = wasm.nodeindex_index(this.ptr);
        return ret >>> 0;
    }
}
/**
*/
export class Obstacle {

    static __wrap(ptr) {
        const obj = Object.create(Obstacle.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_obstacle_free(ptr);
    }
    /**
    * @returns {boolean}
    */
    get is_convex() {
        const ret = wasm.__wbg_get_obstacle_is_convex(this.ptr);
        return ret !== 0;
    }
    /**
    * @param {boolean} arg0
    */
    set is_convex(arg0) {
        wasm.__wbg_set_obstacle_is_convex(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get next_obstacle() {
        const ret = wasm.__wbg_get_obstacle_next_obstacle(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set next_obstacle(arg0) {
        wasm.__wbg_set_obstacle_next_obstacle(this.ptr, arg0);
    }
    /**
    * @returns {Vector2}
    */
    get point_() {
        const ret = wasm.__wbg_get_obstacle_point_(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} arg0
    */
    set point_(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_obstacle_point_(this.ptr, ptr0);
    }
    /**
    * @returns {number}
    */
    get prev_obstacle() {
        const ret = wasm.__wbg_get_obstacle_prev_obstacle(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set prev_obstacle(arg0) {
        wasm.__wbg_set_obstacle_prev_obstacle(this.ptr, arg0);
    }
    /**
    * @returns {Vector2}
    */
    get unit_dir() {
        const ret = wasm.__wbg_get_obstacle_unit_dir(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} arg0
    */
    set unit_dir(arg0) {
        _assertClass(arg0, Vector2);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_obstacle_unit_dir(this.ptr, ptr0);
    }
    /**
    * @returns {number}
    */
    get id_() {
        const ret = wasm.__wbg_get_obstacle_id_(this.ptr);
        return ret >>> 0;
    }
    /**
    * @param {number} arg0
    */
    set id_(arg0) {
        wasm.__wbg_set_obstacle_id_(this.ptr, arg0);
    }
    /**
    * @returns {Obstacle}
    */
    static default() {
        const ret = wasm.obstacle_default();
        return Obstacle.__wrap(ret);
    }
}
/**
*/
export class RVOSimulator {

    static __wrap(ptr) {
        const obj = Object.create(RVOSimulator.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_rvosimulator_free(ptr);
    }
    /**
    * @returns {number}
    */
    get global_time() {
        const ret = wasm.__wbg_get_rvosimulator_global_time(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set global_time(arg0) {
        wasm.__wbg_set_rvosimulator_global_time(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get time_step() {
        const ret = wasm.__wbg_get_rvosimulator_time_step(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set time_step(arg0) {
        wasm.__wbg_set_rvosimulator_time_step(this.ptr, arg0);
    }
    /**
    * @returns {RVOSimulator}
    */
    static default() {
        const ret = wasm.rvosimulator_default();
        return RVOSimulator.__wrap(ret);
    }
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
    static new(max_obstacle, time_step, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity) {
        _assertClass(velocity, Vector2);
        const ret = wasm.rvosimulator_new(max_obstacle, time_step, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity.ptr);
        return RVOSimulator.__wrap(ret);
    }
    /**
    * @param {Vector2} position
    * @returns {number}
    */
    add_agent(position) {
        _assertClass(position, Vector2);
        const ret = wasm.rvosimulator_add_agent(this.ptr, position.ptr);
        return ret >>> 0;
    }
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
    add_agent2(position, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity) {
        _assertClass(position, Vector2);
        _assertClass(velocity, Vector2);
        const ret = wasm.rvosimulator_add_agent2(this.ptr, position.ptr, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity.ptr);
        return ret >>> 0;
    }
    /**
    * @param {Vertices} vertices
    * @returns {number}
    */
    add_obstacle(vertices) {
        _assertClass(vertices, Vertices);
        var ptr0 = vertices.__destroy_into_raw();
        const ret = wasm.rvosimulator_add_obstacle(this.ptr, ptr0);
        return ret >>> 0;
    }
    /**
    */
    do_step() {
        wasm.rvosimulator_do_step(this.ptr);
    }
    /**
    * @param {number} agent_no
    * @param {number} neighbor_no
    * @returns {number}
    */
    get_agent_agent_neighbor(agent_no, neighbor_no) {
        const ret = wasm.rvosimulator_get_agent_agent_neighbor(this.ptr, agent_no, neighbor_no);
        return ret >>> 0;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_max_neighbors(agent_no) {
        const ret = wasm.rvosimulator_get_agent_max_neighbors(this.ptr, agent_no);
        return ret >>> 0;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_max_speed(agent_no) {
        const ret = wasm.rvosimulator_get_agent_max_speed(this.ptr, agent_no);
        return ret;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_neighbor_dist(agent_no) {
        const ret = wasm.rvosimulator_get_agent_neighbor_dist(this.ptr, agent_no);
        return ret;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_num_agent_neighbors(agent_no) {
        const ret = wasm.rvosimulator_get_agent_num_agent_neighbors(this.ptr, agent_no);
        return ret >>> 0;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_num_obstacle_neighbors(agent_no) {
        const ret = wasm.rvosimulator_get_agent_num_obstacle_neighbors(this.ptr, agent_no);
        return ret >>> 0;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_num_orcalines(agent_no) {
        const ret = wasm.rvosimulator_get_agent_num_orcalines(this.ptr, agent_no);
        return ret >>> 0;
    }
    /**
    * @param {number} agent_no
    * @param {number} neighbor_no
    * @returns {number}
    */
    get_agent_obstacle_neighbor(agent_no, neighbor_no) {
        const ret = wasm.rvosimulator_get_agent_obstacle_neighbor(this.ptr, agent_no, neighbor_no);
        return ret >>> 0;
    }
    /**
    * @param {number} agent_no
    * @param {number} line_no
    * @returns {Line}
    */
    get_agent_orcaline(agent_no, line_no) {
        const ret = wasm.rvosimulator_get_agent_orcaline(this.ptr, agent_no, line_no);
        return Line.__wrap(ret);
    }
    /**
    * @param {number} agent_no
    * @returns {Vector2}
    */
    get_agent_position(agent_no) {
        const ret = wasm.rvosimulator_get_agent_position(this.ptr, agent_no);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {number} agent_no
    * @returns {Vector2}
    */
    get_agent_pref_velocity(agent_no) {
        const ret = wasm.rvosimulator_get_agent_pref_velocity(this.ptr, agent_no);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_radius(agent_no) {
        const ret = wasm.rvosimulator_get_agent_radius(this.ptr, agent_no);
        return ret;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_time_horizon(agent_no) {
        const ret = wasm.rvosimulator_get_agent_time_horizon(this.ptr, agent_no);
        return ret;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agent_time_horizon_obst(agent_no) {
        const ret = wasm.rvosimulator_get_agent_time_horizon_obst(this.ptr, agent_no);
        return ret;
    }
    /**
    * @param {number} agent_no
    * @returns {Vector2}
    */
    get_agent_velocity(agent_no) {
        const ret = wasm.rvosimulator_get_agent_velocity(this.ptr, agent_no);
        return Vector2.__wrap(ret);
    }
    /**
    * @returns {number}
    */
    get_global_time() {
        const ret = wasm.__wbg_get_rvosimulator_global_time(this.ptr);
        return ret;
    }
    /**
    * @returns {number}
    */
    get_num_agents() {
        const ret = wasm.rvosimulator_get_num_agents(this.ptr);
        return ret >>> 0;
    }
    /**
    * @returns {number}
    */
    get_num_obstacle_vertices() {
        const ret = wasm.rvosimulator_get_num_obstacle_vertices(this.ptr);
        return ret >>> 0;
    }
    /**
    * @param {number} vertex_no
    * @returns {Vector2}
    */
    get_obstacle_vertex(vertex_no) {
        const ret = wasm.rvosimulator_get_obstacle_vertex(this.ptr, vertex_no);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {number} vertex_no
    * @returns {number}
    */
    get_next_obstacle_vertex_no(vertex_no) {
        const ret = wasm.rvosimulator_get_next_obstacle_vertex_no(this.ptr, vertex_no);
        return ret >>> 0;
    }
    /**
    * @param {number} vertex_no
    * @returns {number}
    */
    get_prev_obstacle_vertex_no(vertex_no) {
        const ret = wasm.rvosimulator_get_prev_obstacle_vertex_no(this.ptr, vertex_no);
        return ret >>> 0;
    }
    /**
    * @returns {number}
    */
    get_time_step() {
        const ret = wasm.__wbg_get_rvosimulator_time_step(this.ptr);
        return ret;
    }
    /**
    * @param {number} agent_no
    * @returns {number}
    */
    get_agents(agent_no) {
        const ret = wasm.rvosimulator_get_agents(this.ptr, agent_no);
        return ret;
    }
    /**
    */
    static process_obstacles() {
        wasm.rvosimulator_process_obstacles();
    }
    /**
    * @param {number} neighbor_dist
    * @param {number} max_neighbors
    * @param {number} time_horizon
    * @param {number} time_horizon_obst
    * @param {number} radius
    * @param {number} max_speed
    * @param {Vector2} velocity
    */
    set_agent_defaults(neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity) {
        _assertClass(velocity, Vector2);
        wasm.rvosimulator_set_agent_defaults(this.ptr, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity.ptr);
    }
    /**
    * @param {number} agent_no
    * @param {number} max_neighbors
    */
    set_agent_max_neighbors(agent_no, max_neighbors) {
        wasm.rvosimulator_set_agent_max_neighbors(this.ptr, agent_no, max_neighbors);
    }
    /**
    * @param {number} agent_no
    * @param {number} max_speed
    */
    set_agent_max_speed(agent_no, max_speed) {
        wasm.rvosimulator_set_agent_max_speed(this.ptr, agent_no, max_speed);
    }
    /**
    * @param {number} agent_no
    * @param {number} neighbor_dist
    */
    set_agent_neighbor_dist(agent_no, neighbor_dist) {
        wasm.rvosimulator_set_agent_neighbor_dist(this.ptr, agent_no, neighbor_dist);
    }
    /**
    * @param {number} agent_no
    * @param {Vector2} position
    */
    set_agent_position(agent_no, position) {
        _assertClass(position, Vector2);
        wasm.rvosimulator_set_agent_position(this.ptr, agent_no, position.ptr);
    }
    /**
    * @param {number} agent_no
    * @param {Vector2} pref_velocity
    */
    set_agent_pref_velocity(agent_no, pref_velocity) {
        _assertClass(pref_velocity, Vector2);
        wasm.rvosimulator_set_agent_pref_velocity(this.ptr, agent_no, pref_velocity.ptr);
    }
    /**
    * @param {number} agent_no
    * @param {number} radius
    */
    set_agent_radius(agent_no, radius) {
        wasm.rvosimulator_set_agent_radius(this.ptr, agent_no, radius);
    }
    /**
    * @param {number} agent_no
    * @param {number} time_horizon
    */
    set_agent_time_horizon(agent_no, time_horizon) {
        wasm.rvosimulator_set_agent_time_horizon(this.ptr, agent_no, time_horizon);
    }
    /**
    * @param {number} agent_no
    * @param {number} time_horizon_obst
    */
    set_agent_time_horizon_obst(agent_no, time_horizon_obst) {
        wasm.rvosimulator_set_agent_time_horizon_obst(this.ptr, agent_no, time_horizon_obst);
    }
    /**
    * @param {number} agent_no
    * @param {Vector2} velocity
    */
    set_agent_velocity(agent_no, velocity) {
        _assertClass(velocity, Vector2);
        wasm.rvosimulator_set_agent_velocity(this.ptr, agent_no, velocity.ptr);
    }
    /**
    * @param {number} time_step
    */
    set_time_step(time_step) {
        wasm.__wbg_set_rvosimulator_time_step(this.ptr, time_step);
    }
}
/**
*/
export class ResultPath {

    static __wrap(ptr) {
        const obj = Object.create(ResultPath.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_resultpath_free(ptr);
    }
    /**
    * @returns {Vector2 | undefined}
    */
    next() {
        const ret = wasm.resultpath_next(this.ptr);
        return ret === 0 ? undefined : Vector2.__wrap(ret);
    }
}
/**
*/
export class TileMap {

    static __wrap(ptr) {
        const obj = Object.create(TileMap.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_tilemap_free(ptr);
    }
    /**
    * @param {number} row
    * @param {number} column
    * @returns {TileMap}
    */
    static new(row, column) {
        const ret = wasm.tilemap_new(row, column);
        return TileMap.__wrap(ret);
    }
    /**
    * @param {NodeIndex} index
    * @param {number} obstacle
    */
    set_obstacle(index, obstacle) {
        _assertClass(index, NodeIndex);
        var ptr0 = index.__destroy_into_raw();
        wasm.tilemap_set_obstacle(this.ptr, ptr0, obstacle);
    }
}
/**
*/
export class Vector2 {

    static __wrap(ptr) {
        const obj = Object.create(Vector2.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_vector2_free(ptr);
    }
    /**
    * @returns {number}
    */
    get x() {
        const ret = wasm.__wbg_get_vector2_x(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set x(arg0) {
        wasm.__wbg_set_vector2_x(this.ptr, arg0);
    }
    /**
    * @returns {number}
    */
    get y() {
        const ret = wasm.__wbg_get_vector2_y(this.ptr);
        return ret;
    }
    /**
    * @param {number} arg0
    */
    set y(arg0) {
        wasm.__wbg_set_vector2_y(this.ptr, arg0);
    }
    /**
    * @returns {Vector2}
    */
    static default() {
        const ret = wasm.vector2_default();
        return Vector2.__wrap(ret);
    }
    /**
    * @param {number} x
    * @param {number} y
    * @returns {Vector2}
    */
    static new(x, y) {
        const ret = wasm.vector2_new(x, y);
        return Vector2.__wrap(ret);
    }
    /**
    * @returns {number}
    */
    x() {
        const ret = wasm.__wbg_get_vector2_x(this.ptr);
        return ret;
    }
    /**
    * @returns {number}
    */
    y() {
        const ret = wasm.__wbg_get_vector2_y(this.ptr);
        return ret;
    }
    /**
    * @param {Vector2} other
    * @returns {Vector2}
    */
    add(other) {
        _assertClass(other, Vector2);
        const ret = wasm.vector2_add(this.ptr, other.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} other
    * @returns {Vector2}
    */
    sub(other) {
        _assertClass(other, Vector2);
        const ret = wasm.vector2_sub(this.ptr, other.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} other
    * @returns {number}
    */
    mul(other) {
        _assertClass(other, Vector2);
        const ret = wasm.vector2_mul(this.ptr, other.ptr);
        return ret;
    }
    /**
    * @param {number} other
    * @returns {Vector2}
    */
    mul_number(other) {
        const ret = wasm.vector2_mul_number(this.ptr, other);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {number} other
    * @returns {Vector2}
    */
    div(other) {
        const ret = wasm.vector2_div(this.ptr, other);
        return Vector2.__wrap(ret);
    }
    /**
    * @returns {Vector2}
    */
    neg() {
        const ret = wasm.vector2_neg(this.ptr);
        return Vector2.__wrap(ret);
    }
    /**
    * @param {Vector2} v
    * @returns {number}
    */
    static abs(v) {
        _assertClass(v, Vector2);
        const ret = wasm.vector2_abs(v.ptr);
        return ret;
    }
    /**
    * @param {Vector2} v
    * @returns {number}
    */
    static abs_sq(v) {
        _assertClass(v, Vector2);
        const ret = wasm.vector2_abs_sq(v.ptr);
        return ret;
    }
    /**
    * @param {Vector2} v1
    * @param {Vector2} v2
    * @returns {number}
    */
    static det(v1, v2) {
        _assertClass(v1, Vector2);
        _assertClass(v2, Vector2);
        const ret = wasm.vector2_det(v1.ptr, v2.ptr);
        return ret;
    }
    /**
    * @param {Vector2} vector
    * @returns {Vector2}
    */
    static normalize(vector) {
        _assertClass(vector, Vector2);
        const ret = wasm.vector2_normalize(vector.ptr);
        return Vector2.__wrap(ret);
    }
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
    static dist_sq_point_line_segment(a, b, c) {
        _assertClass(a, Vector2);
        _assertClass(b, Vector2);
        _assertClass(c, Vector2);
        const ret = wasm.vector2_dist_sq_point_line_segment(a.ptr, b.ptr, c.ptr);
        return ret;
    }
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
    static left_of(a, b, c) {
        _assertClass(a, Vector2);
        _assertClass(b, Vector2);
        _assertClass(c, Vector2);
        const ret = wasm.vector2_left_of(a.ptr, b.ptr, c.ptr);
        return ret;
    }
    /**
    *
    *     * \brief      Computes the square of a float.
    *     * \param      a               The float to be squared.
    *     * \return     The square of the float.
    *
    * @param {number} a
    * @returns {number}
    */
    static sqr(a) {
        const ret = wasm.vector2_sqr(a);
        return ret;
    }
}
/**
*/
export class Vertices {

    static __wrap(ptr) {
        const obj = Object.create(Vertices.prototype);
        obj.ptr = ptr;

        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.ptr;
        this.ptr = 0;

        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_vertices_free(ptr);
    }
    /**
    * @returns {Vertices}
    */
    static new() {
        const ret = wasm.vertices_new();
        return Vertices.__wrap(ret);
    }
    /**
    * @param {Vector2} vertex
    */
    add(vertex) {
        _assertClass(vertex, Vector2);
        var ptr0 = vertex.__destroy_into_raw();
        wasm.vertices_add(this.ptr, ptr0);
    }
    /**
    * @param {number} index
    * @returns {Vector2}
    */
    get(index) {
        const ret = wasm.vertices_get(this.ptr, index);
        return Vector2.__wrap(ret);
    }
    /**
    * @returns {number}
    */
    len() {
        const ret = wasm.vertices_len(this.ptr);
        return ret >>> 0;
    }
}

async function load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);

            } catch (e) {
                if (module.headers.get('Content-Type') != 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else {
                    throw e;
                }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);

    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };

        } else {
            return instance;
        }
    }
}

function getImports() {
    const imports = {};
    imports.wbg = {};
    imports.wbg.__wbindgen_throw = function(arg0, arg1) {
        throw new Error(getStringFromWasm0(arg0, arg1));
    };

    return imports;
}

function initMemory(imports, maybe_memory) {

}

function finalizeInit(instance, module) {
    wasm = instance.exports;
    init.__wbindgen_wasm_module = module;
    cachedUint8Memory0 = null;


    return wasm;
}

function initSync(module) {
    const imports = getImports();

    initMemory(imports);

    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }

    const instance = new WebAssembly.Instance(module, imports);

    return finalizeInit(instance, module);
}

async function init(input) {
    if (typeof input === 'undefined') {
        input = new URL('pi_orca_bg.wasm', import.meta.url);
    }
    const imports = getImports();

    if (typeof input === 'string' || (typeof Request === 'function' && input instanceof Request) || (typeof URL === 'function' && input instanceof URL)) {
        input = fetch(input);
    }

    initMemory(imports);

    const { instance, module } = await load(await input, imports);

    return finalizeInit(instance, module);
}

export { initSync }
export default init;
