use crate::controller::{Controller, FdmState, PilotInput};
use crate::math::*;
use crate::stabilizer::Stabilizer;
use std::any::Any;

/// Demo outer-loop: hold altitude + heading, level attitude. Uses the built-in stabilizer.
pub struct Autopilot {
    pub alt_target: f64,
    pub heading: f64,
    pub hover: f64,
    pub kp_att: f64,
    pub kp_yaw: f64,
    pub kd_yaw: f64,
    pub kp_climb: f64,
    int_alt: f64,
    stab: Stabilizer,
}

impl Autopilot {
    pub fn new(alt_target: f64, heading: f64) -> Self {
        Autopilot {
            alt_target,
            heading,
            hover: 0.70,
            kp_att: 4.0,
            kp_yaw: 2.0,
            kd_yaw: 1.5,
            kp_climb: 0.15,
            int_alt: 0.0,
            stab: Stabilizer::new(),
        }
    }

    pub fn set_target(&mut self, alt: f64, heading: f64) {
        self.alt_target = alt;
        self.heading = heading;
    }
}

impl Controller for Autopilot {
    fn reset(&mut self) {
        self.int_alt = 0.0;
        self.stab.reset();
    }
    fn update(&mut self, dt: f64, s: &FdmState, _pilot: &PilotInput) -> [f64; 4] {
        let (phi, theta, psi) = (s.euler[0], s.euler[1], s.euler[2]);
        let max_pq = self.stab.max_rate_pq;
        let max_r = self.stab.max_rate_r;

        let p_des = (self.kp_att * (0.0 - phi)).clamp(-max_pq, max_pq);
        let q_des = (self.kp_att * (0.0 - theta)).clamp(-max_pq, max_pq);
        let r_des = (self.kp_yaw * wrap_pi(self.heading - psi) - self.kd_yaw * s.omega[2])
            .clamp(-max_r, max_r);

        let e_alt = self.alt_target - s.alt;
        self.int_alt = (self.int_alt + e_alt * dt).clamp(-5.0, 5.0);
        let climb_ref = (0.8 * e_alt + 0.3 * self.int_alt).clamp(-3.0, 3.0);
        let vup = -s.vel_ned[2];
        let thr = (self.hover + self.kp_climb * (climb_ref - vup)).clamp(0.0, 1.0);

        self.stab.mix_rates(dt, thr, [p_des, q_des, r_des], s.omega)
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
