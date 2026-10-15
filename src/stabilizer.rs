use crate::controller::{Controller, FdmState, PilotInput};
use crate::math::*;
use crate::propulsion::mix;
use std::any::Any;

/// Built-in rate-mode stabilizer: pilot stick -> desired body rates -> PI -> motor mixer.
pub struct Stabilizer {
    pub max_rate_pq: f64,
    pub max_rate_r: f64,
    pub kp: f64,
    pub ki: f64,
    pub limit: f64,
    int: [f64; 3],
}

impl Stabilizer {
    pub fn new() -> Self {
        Stabilizer {
            max_rate_pq: 4.0,
            max_rate_r: 2.0,
            kp: 0.09,
            ki: 0.05,
            limit: 0.3,
            int: [0.0; 3],
        }
    }

    /// Build from `[max_rate_pq, max_rate_r, kp, ki, limit]`.
    pub fn from_cfg(c: [f64; 5]) -> Self {
        Stabilizer {
            max_rate_pq: c[0],
            max_rate_r: c[1],
            kp: c[2],
            ki: c[3],
            limit: c[4],
            int: [0.0; 3],
        }
    }

    pub fn rates(&self, pilot: &PilotInput) -> (f64, V3) {
        let thr = pilot.throttle.clamp(0.0, 1.0);
        let rates = [
            pilot.roll.clamp(-1.0, 1.0) * self.max_rate_pq,
            pilot.pitch.clamp(-1.0, 1.0) * self.max_rate_pq,
            pilot.yaw.clamp(-1.0, 1.0) * self.max_rate_r,
        ];
        (thr, rates)
    }

    pub fn mix_rates(&mut self, dt: f64, thr: f64, rates: V3, w: V3) -> [f64; 4] {
        let e = sub(rates, w);
        for i in 0..3 {
            self.int[i] = (self.int[i] + e[i] * dt).clamp(-2.0, 2.0);
        }
        let u0 = (self.kp * e[0] + self.ki * self.int[0]).clamp(-self.limit, self.limit);
        let u1 = (self.kp * e[1] + self.ki * self.int[1]).clamp(-self.limit, self.limit);
        let u2 = (self.kp * e[2] + self.ki * self.int[2]).clamp(-self.limit, self.limit);
        mix(thr, u0, u1, u2)
    }
}

impl Controller for Stabilizer {
    fn reset(&mut self) {
        self.int = [0.0; 3];
    }
    fn update(&mut self, dt: f64, state: &FdmState, pilot: &PilotInput) -> [f64; 4] {
        let (thr, rates) = self.rates(pilot);
        self.mix_rates(dt, thr, rates, state.omega)
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
