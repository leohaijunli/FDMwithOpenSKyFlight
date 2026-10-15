/**
 * Thin wrapper around the Rust RFDM flight dynamics core compiled to
 * wasm32-unknown-unknown (bare C ABI, no wasm-bindgen).
 *
 * State layout returned by `writeState()` (Float64Array, length 24):
 *   0      time (s)
 *   1..4   pos_ned  north, east, down (m)
 *   4..7   vel_ned  north, east, down (m/s)
 *   7..10  vel_body u, v, w (m/s)
 *   10..13 omega    p, q, r (rad/s)
 *   13..16 accel    body specific force (m/s^2)
 *   16..20 quat     w, x, y, z (body -> NED)
 *   20..23 euler    roll, pitch, yaw (rad)
 *   23     alt      height AGL (m)
 */

const WASM_URL = new URL('./rfdm.wasm', import.meta.url);

export const STATE_LEN = 24;
export const RPM_LEN = 4;

export const IDX = {
  time: 0,
  posN: 1,
  posE: 2,
  posD: 3,
  velN: 4,
  velE: 5,
  velD: 6,
  u: 7,
  v: 8,
  w: 9,
  p: 10,
  q: 11,
  r: 12,
  ax: 13,
  ay: 14,
  az: 15,
  qw: 16,
  qx: 17,
  qy: 18,
  qz: 19,
  roll: 20,
  pitch: 21,
  yaw: 22,
  alt: 23,
};

export default class Rfdm {
  static async load() {
    const res = await fetch(WASM_URL);
    if (!res.ok) throw new Error(`Failed to fetch rfdm.wasm (${res.status})`);
    const bytes = await res.arrayBuffer();
    const { instance } = await WebAssembly.instantiate(bytes, {});
    return new Rfdm(instance.exports);
  }

  constructor(exports) {
    this._e = exports;
    this._sim = exports.rfdm_new();
    if (!this._sim) throw new Error('rfdm_new returned null');
    this._scratch = exports.rfdm_scratch();
    this._buffer = null;
    this._f64 = null;
    this.state = new Float64Array(STATE_LEN);
    this.rpm = new Float64Array(RPM_LEN);
  }

  _view() {
    const buffer = this._e.memory.buffer;
    if (buffer !== this._buffer) {
      this._buffer = buffer;
      this._f64 = new Float64Array(buffer, this._scratch, STATE_LEN + RPM_LEN);
    }
    return this._f64;
  }

  reset() {
    this._e.rfdm_reset(this._sim);
  }

  /** Advance one fixed step. Sticks: throttle [0,1], roll/pitch/yaw [-1,1]. */
  step(dt, throttle, roll, pitch, yaw) {
    this._e.rfdm_step(this._sim, dt, throttle, roll, pitch, yaw);
  }

  /** Advance one fixed step driving the 4 motors directly (order [FR, AL, FL, AR]). */
  stepMotors(dt, motors) {
    this._e.rfdm_step_motors(this._sim, dt, motors[0], motors[1], motors[2], motors[3]);
  }

  /**
   * Reconfigure the plant (mass, inertia, motors, props, rotors, aero, ground,
   * initial state, stabilizer gains) from a plain object and reset the sim.
   * Any omitted field keeps its current value.
   */
  setConfig(config) {
    const json = JSON.stringify(config);
    const enc = new TextEncoder().encode(json);
    const ptr = this._e.rfdm_alloc(enc.length);
    new Uint8Array(this._e.memory.buffer, ptr, enc.length).set(enc);
    const ret = this._e.rfdm_set_config(this._sim, ptr, enc.length);
    this._e.rfdm_free_bytes(ptr, enc.length);
    if (ret !== 0) throw new Error(`rfdm_set_config failed (${ret})`);
  }

  /** Refresh and return the cached state array (see layout above). */
  readState() {
    this._e.rfdm_write_state(this._sim, this._scratch);
    const view = this._view();
    this.state.set(view.subarray(0, STATE_LEN));
    return this.state;
  }

  /** Refresh and return the cached rotor RPM array [FR, AL, FL, AR]. */
  readRpm() {
    this._e.rfdm_write_rpm(this._sim, this._scratch + STATE_LEN * 8);
    const view = this._view();
    this.rpm.set(view.subarray(STATE_LEN, STATE_LEN + RPM_LEN));
    return this.rpm;
  }
}
