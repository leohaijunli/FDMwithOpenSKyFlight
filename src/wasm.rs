use crate::config::f450;
use crate::fdm::{Fdm, State};
use crate::sim::Sim;
use crate::stabilizer::Stabilizer;

/// Number of `f64` values written by [`rfdm_write_state`].
pub const STATE_LEN: usize = 24;

/// Number of `f64` values written by [`rfdm_write_rpm`].
pub const RPM_LEN: usize = 4;

static mut SCRATCH: [f64; STATE_LEN + RPM_LEN] = [0.0; STATE_LEN + RPM_LEN];

/// Pointer to shared scratch memory: `state` at offset 0, `rpm` at offset 24.
#[no_mangle]
pub extern "C" fn rfdm_scratch() -> *mut f64 {
    std::ptr::addr_of_mut!(SCRATCH) as *mut f64
}

fn initial_state() -> State {
    State {
        p_ned: [0.0, 0.0, -5.0],
        v_ned: [0.0; 3],
        w_b: [0.0; 3],
        q: [1.0, 0.0, 0.0, 0.0],
    }
}

/// Allocate a new simulation (F450 plant + built-in rate stabilizer).
#[no_mangle]
pub extern "C" fn rfdm_new() -> *mut Sim {
    let fdm = Fdm::new(f450(), initial_state());
    Box::into_raw(Box::new(Sim::new(fdm, Box::new(Stabilizer::new()))))
}

/// Destroy a simulation created by [`rfdm_new`].
#[no_mangle]
pub extern "C" fn rfdm_free(ptr: *mut Sim) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

/// Reset the plant to the takeoff state and clear controller integrators.
#[no_mangle]
pub extern "C" fn rfdm_reset(ptr: *mut Sim) {
    if ptr.is_null() {
        return;
    }
    let sim = unsafe { &mut *ptr };
    sim.set_controller(Box::new(Stabilizer::new()));
    sim.fdm.reset(initial_state());
}

/// Advance one physics step.
///
/// Sticks are normalized: `throttle` in [0, 1], `roll`/`pitch`/`yaw` in [-1, 1].
#[no_mangle]
pub extern "C" fn rfdm_step(
    ptr: *mut Sim,
    dt: f64,
    throttle: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
) {
    if ptr.is_null() {
        return;
    }
    let sim = unsafe { &mut *ptr };
    let pilot = crate::controller::PilotInput { throttle, roll, pitch, yaw };
    sim.step(dt, &pilot);
}

/// Advance one physics step driving the 4 motors directly, bypassing the
/// built-in stabilizer. Commands are in [0, 1], order [FR, AL, FL, AR].
#[no_mangle]
pub extern "C" fn rfdm_step_motors(ptr: *mut Sim, dt: f64, m0: f64, m1: f64, m2: f64, m3: f64) {
    if ptr.is_null() {
        return;
    }
    let sim = unsafe { &mut *ptr };
    sim.step_motors(dt, &[m0, m1, m2, m3]);
}

/// Write the current plant state into `out` (must hold [`STATE_LEN`] `f64`s).
///
/// Layout:
/// ```text
/// 0      time
/// 1..4   pos_ned   north, east, down   (m)
/// 4..7   vel_ned   north, east, down   (m/s)
/// 7..10  vel_body  u, v, w             (m/s)
/// 10..13 omega     p, q, r             (rad/s)
/// 13..16 accel     body specific force (m/s^2)
/// 16..20 quat      w, x, y, z           (body -> NED)
/// 20..23 euler     roll, pitch, yaw    (rad)
/// 23     alt       height AGL          (m)
/// ```
#[no_mangle]
pub extern "C" fn rfdm_write_state(ptr: *const Sim, out: *mut f64) {
    if ptr.is_null() || out.is_null() {
        return;
    }
    let sim = unsafe { &*ptr };
    let s = sim.snapshot();
    let o = unsafe { std::slice::from_raw_parts_mut(out, STATE_LEN) };
    o[0] = s.time;
    o[1..4].copy_from_slice(&s.pos_ned);
    o[4..7].copy_from_slice(&s.vel_ned);
    o[7..10].copy_from_slice(&s.vel_body);
    o[10..13].copy_from_slice(&s.omega);
    o[13..16].copy_from_slice(&s.accel_body);
    o[16..20].copy_from_slice(&s.quat);
    o[20..23].copy_from_slice(&s.euler);
    o[23] = s.alt;
}

/// Total rotor RPM as a `[FR, AL, FL, AR]` array written into `out` (4 `f64`s).
#[no_mangle]
pub extern "C" fn rfdm_write_rpm(ptr: *const Sim, out: *mut f64) {
    if ptr.is_null() || out.is_null() {
        return;
    }
    let sim = unsafe { &*ptr };
    let o = unsafe { std::slice::from_raw_parts_mut(out, RPM_LEN) };
    for i in 0..4 {
        o[i] = sim.fdm.aircraft.quad.rotors[i].rps * 60.0;
    }
}
