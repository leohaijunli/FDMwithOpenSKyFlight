use crate::math::*;

#[derive(Clone, Copy)]
pub struct Contact {
    pub pos: V3,
    pub k: f64,
    pub c: f64,
    pub mu_s: f64,
    pub mu_d: f64,
}

pub struct Ground {
    pub contacts: Vec<Contact>,
}

impl Ground {
    pub fn step(&self, p_ned: V3, v_ned: V3, q: Quat, w_b: V3) -> (V3, V3) {
        let mut force = [0.0; 3];
        let mut moment = [0.0; 3];
        for c in &self.contacts {
            let r_ned = qrot(q, c.pos);
            let pc = add(p_ned, r_ned);
            let depth = pc[2];
            if depth <= 0.0 {
                continue;
            }
            let vc = add(v_ned, qrot(q, cross(w_b, c.pos)));
            let vz = vc[2];
            let n = (c.k * depth + c.c * vz).max(0.0);

            let mut f_ned = [0.0, 0.0, -n];
            let vh = [vc[0], vc[1], 0.0];
            let vh_mag = norm(vh);
            if vh_mag > 1e-4 {
                let mu = if vh_mag > 1.0 { c.mu_d } else { c.mu_s };
                let ff = mu * n;
                f_ned = add(f_ned, scale(scale(vh, 1.0 / vh_mag), -ff));
            }
            let f_b = qrot_inv(q, f_ned);
            force = add(force, f_b);
            moment = add(moment, cross(c.pos, f_b));
        }
        (force, moment)
    }
}
