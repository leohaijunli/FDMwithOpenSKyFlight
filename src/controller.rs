use crate::math::*;

/// Read-only snapshot of the plant, handed to a [`Controller`] each step.
#[derive(Clone, Copy)]
pub struct FdmState {
    /// Seconds since reset.
    pub time: f64,
    /// Position in local NED (m): north, east, down. Origin = takeoff point, ground at down=0.
    pub pos_ned: V3,
    /// Velocity in local NED (m/s).
    pub vel_ned: V3,
    /// Velocity in body frame (m/s): u forward, v right, w down.
    pub vel_body: V3,
    /// Angular velocity in body frame (rad/s): p roll, q pitch, r yaw.
    pub omega: V3,
    /// Specific force in body frame (m/s^2): non-gravitational force / mass.
    /// This is exactly what an ideal accelerometer measures.
    pub accel_body: V3,
    /// Body -> NED quaternion [w, x, y, z].
    pub quat: Quat,
    /// Attitude Euler angles (rad): phi roll, theta pitch, psi yaw.
    pub euler: V3,
    /// Height above ground (m).
    pub alt: f64,
}

/// Pilot / autopilot command, dimensionless, normalized to [-1, 1] (throttle to [0, 1]).
#[derive(Clone, Copy)]
pub struct PilotInput {
    pub throttle: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Default for PilotInput {
    fn default() -> Self {
        PilotInput { throttle: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0 }
    }
}

/// Pluggable control law. Implement this to replace the built-in stabilizer.
///
/// Contract:
/// - Called once per `dt` by [`crate::sim::Sim`], with the plant state *before* the step.
/// - Returns 4 motor throttle commands in [0, 1], in rotor order
///   [front-right, aft-left, front-left, aft-right].
pub trait Controller {
    fn reset(&mut self);
    fn update(&mut self, dt: f64, state: &FdmState, pilot: &PilotInput) -> [f64; 4];
}

pub fn build_fdm_state(
    time: f64,
    pos_ned: V3,
    vel_ned: V3,
    omega: V3,
    accel_body: V3,
    quat: Quat,
) -> FdmState {
    let vel_body = qrot_inv(quat, vel_ned);
    let (phi, theta, psi) = euler_from_q(quat);
    FdmState {
        time,
        pos_ned,
        vel_ned,
        vel_body,
        omega,
        accel_body,
        quat,
        euler: [phi, theta, psi],
        alt: -pos_ned[2],
    }
}
