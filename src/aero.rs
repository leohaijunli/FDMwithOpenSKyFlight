use crate::math::*;

pub struct Aero {
    pub s: f64,
    pub cd: f64,
}

impl Aero {
    pub fn step(&self, v_b: V3, rho: f64) -> V3 {
        let v = norm(v_b);
        if v < 1e-6 {
            return [0.0; 3];
        }
        let qbar = 0.5 * rho * v * v;
        let drag = qbar * self.s * self.cd;
        scale(scale(v_b, 1.0 / v), -drag)
    }
}
