use crate::config::f450;
use crate::fdm::{Fdm, State};
use crate::json::Json;
use crate::math::q_from_euler;
use crate::sim::Sim;
use crate::stabilizer::Stabilizer;

/// Number of `f64` values written by [`rfdm_write_state`].
pub const STATE_LEN: usize = 24;

/// Number of `f64` values written by [`rfdm_write_rpm`].
pub const RPM_LEN: usize = 4;

static mut SCRATCH: [f64; STATE_LEN + RPM_LEN] = [0.0; STATE_LEN + RPM_LEN];

const CONFIG_BUF_LEN: usize = 4096;
static mut CONFIG_BUF: [u8; CONFIG_BUF_LEN] = [0; CONFIG_BUF_LEN];
static mut INIT_ALT: f64 = 5.0;
static mut INIT_PSI: f64 = 0.0;
static mut STAB_CFG: [f64; 5] = [4.0, 2.0, 0.09, 0.05, 0.3];

/// Pointer to shared scratch memory: `state` at offset 0, `rpm` at offset 24.
#[no_mangle]
pub extern "C" fn rfdm_scratch() -> *mut f64 {
    std::ptr::addr_of_mut!(SCRATCH) as *mut f64
}

fn initial_state() -> State {
    let (alt, psi) = unsafe { (INIT_ALT, INIT_PSI) };
    State {
        p_ned: [0.0, 0.0, -alt],
        v_ned: [0.0; 3],
        w_b: [0.0; 3],
        q: q_from_euler(0.0, 0.0, psi),
    }
}

fn new_stabilizer() -> Stabilizer {
    Stabilizer::from_cfg(unsafe { STAB_CFG })
}

/// Allocate a new simulation (F450 plant + built-in rate stabilizer).
#[no_mangle]
pub extern "C" fn rfdm_new() -> *mut Sim {
    let fdm = Fdm::new(f450(), initial_state());
    Box::into_raw(Box::new(Sim::new(fdm, Box::new(new_stabilizer()))))
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
    sim.set_controller(Box::new(new_stabilizer()));
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

fn set_rotor_motor(rotor: &mut crate::propulsion::Rotor, obj: &Json) {
    if let Some(v) = obj.get("kv").and_then(Json::num) {
        rotor.motor.kv = v;
    }
    if let Some(v) = obj.get("resistance").and_then(Json::num) {
        rotor.motor.resistance = v;
    }
    if let Some(v) = obj.get("i0").and_then(Json::num) {
        rotor.motor.i0 = v;
    }
    if let Some(v) = obj.get("vmax").and_then(Json::num) {
        rotor.motor.vmax = v;
    }
}

fn set_rotor_prop(rotor: &mut crate::propulsion::Rotor, obj: &Json) {
    if let Some(v) = obj.get("diam").and_then(Json::num) {
        rotor.prop.diam = v;
    }
    if let Some(v) = obj.get("ixx").and_then(Json::num) {
        rotor.prop.ixx = v;
    }
    if let Some(v) = obj.get("senseMult").and_then(Json::num) {
        rotor.prop.sense_multiplier = v;
    }
}

fn apply_config(ptr: *mut Sim, json: &Json) {
    {
        let sim = unsafe { &mut *ptr };
        let ac = &mut sim.fdm.aircraft;
        if let Some(v) = json.get("mass").and_then(Json::num) {
            ac.mass = v;
        }
        if let Some(arr) = json.get("inertia").and_then(Json::arr) {
            for i in 0..arr.len().min(3) {
                if let Some(v) = arr[i].num() {
                    ac.inertia[i] = v;
                }
            }
        }
        if let Some(m) = json.get("motor") {
            for r in ac.quad.rotors.iter_mut() {
                set_rotor_motor(r, m);
            }
        }
        if let Some(p) = json.get("prop") {
            for r in ac.quad.rotors.iter_mut() {
                set_rotor_prop(r, p);
            }
        }
        if let Some(rotors) = json.get("rotors").and_then(Json::arr) {
            for (i, item) in rotors.iter().enumerate() {
                if i >= ac.quad.rotors.len() {
                    break;
                }
                let rotor = &mut ac.quad.rotors[i];
                if let Some(v) = item.get("x").and_then(Json::num) {
                    rotor.pos[0] = v;
                }
                if let Some(v) = item.get("y").and_then(Json::num) {
                    rotor.pos[1] = v;
                }
                if let Some(v) = item.get("z").and_then(Json::num) {
                    rotor.pos[2] = v;
                }
                if let Some(v) = item.get("sense").and_then(Json::num) {
                    rotor.sense = v;
                }
            }
        }
        if let Some(aero) = json.get("aero") {
            if let Some(v) = aero.get("s").and_then(Json::num) {
                ac.aero.s = v;
            }
            if let Some(v) = aero.get("cd").and_then(Json::num) {
                ac.aero.cd = v;
            }
        }
        if let Some(g) = json.get("ground") {
            if let Some(v) = g.get("k").and_then(Json::num) {
                for c in ac.ground.contacts.iter_mut() {
                    c.k = v;
                }
            }
            if let Some(v) = g.get("c").and_then(Json::num) {
                for c in ac.ground.contacts.iter_mut() {
                    c.c = v;
                }
            }
            if let Some(v) = g.get("muS").and_then(Json::num) {
                for c in ac.ground.contacts.iter_mut() {
                    c.mu_s = v;
                }
            }
            if let Some(v) = g.get("muD").and_then(Json::num) {
                for c in ac.ground.contacts.iter_mut() {
                    c.mu_d = v;
                }
            }
            if let Some(pts) = g.get("contacts").and_then(Json::arr) {
                for (i, item) in pts.iter().enumerate() {
                    if let Some(p) = item.arr() {
                        if p.len() >= 3 && i < ac.ground.contacts.len() {
                            let p0 = p[0].num().unwrap_or(0.0);
                            let p1 = p[1].num().unwrap_or(0.0);
                            let p2 = p[2].num().unwrap_or(0.0);
                            ac.ground.contacts[i].pos = [p0, p1, p2];
                        }
                    }
                }
            }
        }
    }
    if let Some(init) = json.get("initial") {
        if let Some(v) = init.get("alt").and_then(Json::num) {
            unsafe { INIT_ALT = v };
        }
        if let Some(v) = init.get("psi").and_then(Json::num) {
            unsafe { INIT_PSI = v };
        }
    }
    if let Some(s) = json.get("stabilizer") {
        unsafe {
            if let Some(v) = s.get("maxRatePq").and_then(Json::num) {
                STAB_CFG[0] = v;
            }
            if let Some(v) = s.get("maxRateR").and_then(Json::num) {
                STAB_CFG[1] = v;
            }
            if let Some(v) = s.get("kp").and_then(Json::num) {
                STAB_CFG[2] = v;
            }
            if let Some(v) = s.get("ki").and_then(Json::num) {
                STAB_CFG[3] = v;
            }
            if let Some(v) = s.get("limit").and_then(Json::num) {
                STAB_CFG[4] = v;
            }
        }
        let sim = unsafe { &mut *ptr };
        if let Some(st) = sim.controller.as_any_mut().downcast_mut::<Stabilizer>() {
            st.max_rate_pq = unsafe { STAB_CFG[0] };
            st.max_rate_r = unsafe { STAB_CFG[1] };
            st.kp = unsafe { STAB_CFG[2] };
            st.ki = unsafe { STAB_CFG[3] };
            st.limit = unsafe { STAB_CFG[4] };
        }
    }
}

/// Allocate `len` bytes of linear memory, returning a pointer for JS writes.
#[no_mangle]
pub extern "C" fn rfdm_alloc(len: u32) -> *mut u8 {
    let mut v = Vec::<u8>::with_capacity(len as usize);
    let p = v.as_mut_ptr();
    std::mem::forget(v);
    p
}

/// Free a buffer previously returned by [`rfdm_alloc`].
#[no_mangle]
pub extern "C" fn rfdm_free_bytes(ptr: *mut u8, len: u32) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Vec::from_raw_parts(ptr, len as usize, len as usize));
    }
}

/// Reconfigure the plant (mass, inertia, motors, props, rotors, aero, ground,
/// initial state, stabilizer gains) from a JSON object and reset the sim.
///
/// Returns 0 on success, 1 null args, 2 payload too large, 3 malformed JSON.
#[no_mangle]
pub extern "C" fn rfdm_set_config(ptr: *mut Sim, json: *const u8, len: u32) -> i32 {
    if ptr.is_null() || json.is_null() {
        return 1;
    }
    let len = len as usize;
    if len > CONFIG_BUF_LEN {
        return 2;
    }
    unsafe {
        let src = std::slice::from_raw_parts(json, len);
        CONFIG_BUF[..len].copy_from_slice(src);
    }
    match crate::json::parse(unsafe { &CONFIG_BUF[..len] }) {
        Ok(json) => {
            apply_config(ptr, &json);
            let sim = unsafe { &mut *ptr };
            sim.set_controller(Box::new(new_stabilizer()));
            sim.fdm.reset(initial_state());
            0
        }
        Err(()) => 3,
    }
}
