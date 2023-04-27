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

let cachedInt32Memory0 = null;

function getInt32Memory0() {
    if (cachedInt32Memory0 === null || cachedInt32Memory0.byteLength === 0) {
        cachedInt32Memory0 = new Int32Array(wasm.memory.buffer);
    }
    return cachedInt32Memory0;
}

let cachedFloat64Memory0 = null;

function getFloat64Memory0() {
    if (cachedFloat64Memory0 === null || cachedFloat64Memory0.byteLength === 0) {
        cachedFloat64Memory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64Memory0;
}

let cachedFloat32Memory0 = null;

function getFloat32Memory0() {
    if (cachedFloat32Memory0 === null || cachedFloat32Memory0.byteLength === 0) {
        cachedFloat32Memory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32Memory0;
}
/**
*/
export class ID {

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
        return ret;
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
    * @param {number} _max_obstacle
    * @returns {RVOSimulator}
    */
    static default(_max_obstacle) {
        const ret = wasm.rvosimulator_default(_max_obstacle);
        return RVOSimulator.__wrap(ret);
    }
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
    static new(_max_obstacle, time_step, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity) {
        _assertClass(velocity, Vector2);
        const ret = wasm.rvosimulator_new(_max_obstacle, time_step, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity.ptr);
        return RVOSimulator.__wrap(ret);
    }
    /**
    * @param {Vector2} position
    * @returns {number}
    */
    add_agent(position) {
        _assertClass(position, Vector2);
        const ret = wasm.rvosimulator_add_agent(this.ptr, position.ptr);
        return ret;
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
        return ret;
    }
    /**
    * @param {number} id
    * @returns {boolean}
    */
    remove_agent(id) {
        const ret = wasm.rvosimulator_remove_agent(this.ptr, id);
        return ret !== 0;
    }
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
    add_obstacle(vertices) {
        _assertClass(vertices, Vertices);
        var ptr0 = vertices.__destroy_into_raw();
        const ret = wasm.rvosimulator_add_obstacle(this.ptr, ptr0);
        return ret;
    }
    /**
    * @param {number} id
    * @returns {boolean}
    */
    remove_obstacle(id) {
        const ret = wasm.rvosimulator_remove_obstacle(this.ptr, id);
        return ret !== 0;
    }
    /**
    */
    do_step() {
        wasm.rvosimulator_do_step(this.ptr);
    }
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
    set_agent_defaults(neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity) {
        _assertClass(velocity, Vector2);
        wasm.rvosimulator_set_agent_defaults(this.ptr, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed, velocity.ptr);
    }
    /**
    * @param {number} agent_no
    * @param {number} neighbor_no
    * @returns {number | undefined}
    */
    get_agent_agent_neighbor(agent_no, neighbor_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_agent_neighbor(retptr, this.ptr, agent_no, neighbor_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r2 = getFloat64Memory0()[retptr / 8 + 1];
            return r0 === 0 ? undefined : r2;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_max_neighbors(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_max_neighbors(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getInt32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1 >>> 0;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_max_speed(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_max_speed(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getFloat32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_neighbor_dist(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_neighbor_dist(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getFloat32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_num_agent_neighbors(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_num_agent_neighbors(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getInt32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1 >>> 0;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_num_obstacle_neighbors(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_num_obstacle_neighbors(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getInt32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1 >>> 0;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_num_orcalines(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_num_orcalines(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getInt32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1 >>> 0;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @param {number} neighbor_no
    * @returns {number | undefined}
    */
    get_agent_obstacle_neighbor(agent_no, neighbor_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_obstacle_neighbor(retptr, this.ptr, agent_no, neighbor_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r2 = getFloat64Memory0()[retptr / 8 + 1];
            return r0 === 0 ? undefined : r2;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {Vector2 | undefined}
    */
    get_agent_position(agent_no) {
        const ret = wasm.rvosimulator_get_agent_position(this.ptr, agent_no);
        return ret === 0 ? undefined : Vector2.__wrap(ret);
    }
    /**
    * @param {number} agent_no
    * @returns {Vector2 | undefined}
    */
    get_agent_pref_velocity(agent_no) {
        const ret = wasm.rvosimulator_get_agent_pref_velocity(this.ptr, agent_no);
        return ret === 0 ? undefined : Vector2.__wrap(ret);
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_radius(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_radius(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getFloat32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_time_horizon(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_time_horizon(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getFloat32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {number | undefined}
    */
    get_agent_time_horizon_obst(agent_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_agent_time_horizon_obst(retptr, this.ptr, agent_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r1 = getFloat32Memory0()[retptr / 4 + 1];
            return r0 === 0 ? undefined : r1;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} agent_no
    * @returns {Vector2 | undefined}
    */
    get_agent_velocity(agent_no) {
        const ret = wasm.rvosimulator_get_agent_velocity(this.ptr, agent_no);
        return ret === 0 ? undefined : Vector2.__wrap(ret);
    }
    /**
    * @returns {number}
    */
    get_global_time() {
        const ret = wasm.rvosimulator_get_global_time(this.ptr);
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
    * @returns {Vector2 | undefined}
    */
    get_obstacle_vertex(vertex_no) {
        const ret = wasm.rvosimulator_get_obstacle_vertex(this.ptr, vertex_no);
        return ret === 0 ? undefined : Vector2.__wrap(ret);
    }
    /**
    * @param {number} vertex_no
    * @returns {number | undefined}
    */
    get_next_obstacle_vertex_no(vertex_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_next_obstacle_vertex_no(retptr, this.ptr, vertex_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r2 = getFloat64Memory0()[retptr / 8 + 1];
            return r0 === 0 ? undefined : r2;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @param {number} vertex_no
    * @returns {number | undefined}
    */
    get_prev_obstacle_vertex_no(vertex_no) {
        try {
            const retptr = wasm.__wbindgen_add_to_stack_pointer(-16);
            wasm.rvosimulator_get_prev_obstacle_vertex_no(retptr, this.ptr, vertex_no);
            var r0 = getInt32Memory0()[retptr / 4 + 0];
            var r2 = getFloat64Memory0()[retptr / 8 + 1];
            return r0 === 0 ? undefined : r2;
        } finally {
            wasm.__wbindgen_add_to_stack_pointer(16);
        }
    }
    /**
    * @returns {number}
    */
    get_time_step() {
        const ret = wasm.rvosimulator_get_time_step(this.ptr);
        return ret;
    }
    /**
    * @param {number} agent_no
    * @param {number} max_neighbors
    * @returns {boolean}
    */
    set_agent_max_neighbors(agent_no, max_neighbors) {
        const ret = wasm.rvosimulator_set_agent_max_neighbors(this.ptr, agent_no, max_neighbors);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {number} max_speed
    * @returns {boolean}
    */
    set_agent_max_speed(agent_no, max_speed) {
        const ret = wasm.rvosimulator_set_agent_max_speed(this.ptr, agent_no, max_speed);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {number} neighbor_dist
    * @returns {boolean}
    */
    set_agent_neighbor_dist(agent_no, neighbor_dist) {
        const ret = wasm.rvosimulator_set_agent_neighbor_dist(this.ptr, agent_no, neighbor_dist);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {Vector2} position
    * @returns {boolean}
    */
    set_agent_position(agent_no, position) {
        _assertClass(position, Vector2);
        const ret = wasm.rvosimulator_set_agent_position(this.ptr, agent_no, position.ptr);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {Vector2} pref_velocity
    * @returns {boolean}
    */
    set_agent_pref_velocity(agent_no, pref_velocity) {
        _assertClass(pref_velocity, Vector2);
        const ret = wasm.rvosimulator_set_agent_pref_velocity(this.ptr, agent_no, pref_velocity.ptr);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {number} radius
    * @returns {boolean}
    */
    set_agent_radius(agent_no, radius) {
        const ret = wasm.rvosimulator_set_agent_radius(this.ptr, agent_no, radius);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {number} time_horizon
    * @returns {boolean}
    */
    set_agent_time_horizon(agent_no, time_horizon) {
        const ret = wasm.rvosimulator_set_agent_time_horizon(this.ptr, agent_no, time_horizon);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {number} time_horizon_obst
    * @returns {boolean}
    */
    set_agent_time_horizon_obst(agent_no, time_horizon_obst) {
        const ret = wasm.rvosimulator_set_agent_time_horizon_obst(this.ptr, agent_no, time_horizon_obst);
        return ret !== 0;
    }
    /**
    * @param {number} agent_no
    * @param {Vector2} velocity
    * @returns {boolean}
    */
    set_agent_velocity(agent_no, velocity) {
        _assertClass(velocity, Vector2);
        const ret = wasm.rvosimulator_set_agent_velocity(this.ptr, agent_no, velocity.ptr);
        return ret !== 0;
    }
    /**
    * @param {number} time_step
    */
    set_time_step(time_step) {
        wasm.rvosimulator_set_time_step(this.ptr, time_step);
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
        const ret = wasm.vector2_x(this.ptr);
        return ret;
    }
    /**
    * @returns {number}
    */
    y() {
        const ret = wasm.vector2_y(this.ptr);
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
    cachedFloat32Memory0 = null;
    cachedFloat64Memory0 = null;
    cachedInt32Memory0 = null;
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
