use crate::atmosphere::{isa, G0};
use crate::config::Aircraft;
use crate::controller::{build_fdm_state, FdmState};
use crate::math::*;

const RE: f64 = 6_371_000.0;

#[derive(Clone, Copy)]
pub struct State {
    pub p_ned: V3,
    pub v_ned: V3,
    pub w_b: V3,
    pub q: Quat,
}

struct Hist {
    d: [V3; 3],
}

impl Hist {
    fn new() -> Hist {
        Hist { d: [[0.0; 3]; 3] }
    }
    fn push(&mut self, v: V3) {
        self.d[2] = self.d[1];
        self.d[1] = self.d[0];
        self.d[0] = v;
    }
    fn ab2(&mut self, val: &mut V3, deriv: V3, dt: f64) {
        self.push(deriv);
        let inc = add(scale(self.d[0], 1.5), scale(self.d[1], -0.5));
        *val = add(*val, scale(inc, dt));
    }
    fn ab3(&mut self, val: &mut V3, deriv: V3, dt: f64) {
        self.push(deriv);
        let inc = add(
            add(scale(self.d[0], 23.0), scale(self.d[1], -16.0)),
            scale(self.d[2], 5.0),
        );
        *val = add(*val, scale(inc, dt / 12.0));
    }
}

pub struct Fdm {
    pub aircraft: Aircraft,
    state: State,
    spec_force: V3,
    hist_a: Hist,
    hist_v: Hist,
    time: f64,
}

impl Fdm {
    pub fn new(aircraft: Aircraft, state: State) -> Fdm {
        Fdm {
            aircraft,
            state,
            spec_force: [0.0; 3],
            hist_a: Hist::new(),
            hist_v: Hist::new(),
            time: 0.0,
        }
    }

    pub fn reset(&mut self, state: State) {
        self.state = state;
        self.spec_force = [0.0; 3];
        self.hist_a = Hist::new();
        self.hist_v = Hist::new();
        self.time = 0.0;
    }

    pub fn step(&mut self, dt: f64, motors: &[f64; 4]) {
        let s = self.state;
        let ac = &mut self.aircraft;
        let alt = -s.p_ned[2];
        let rho = isa(alt).rho;
        let vb = qrot_inv(s.q, s.v_ned);

        let (mut f, mut m) = ac.quad.step(dt, motors, vb, s.w_b, rho);
        f = add(f, ac.aero.step(vb, rho));
        let (fg, mg) = ac.ground.step(s.p_ned, s.v_ned, s.q, s.w_b);
        f = add(f, fg);
        m = add(m, mg);

        let g = G0 * (RE / (RE + alt)).powi(2);
        let spec = scale(f, 1.0 / ac.mass);
        let a_ned = add(qrot(s.q, spec), [0.0, 0.0, g]);

        let jw = [
            ac.inertia[0] * s.w_b[0],
            ac.inertia[1] * s.w_b[1],
            ac.inertia[2] * s.w_b[2],
        ];
        let mnet = sub(m, cross(s.w_b, jw));
        let w_dot = [
            mnet[0] / ac.inertia[0],
            mnet[1] / ac.inertia[1],
            mnet[2] / ac.inertia[2],
        ];

        let qd = qmul(s.q, [0.0, s.w_b[0], s.w_b[1], s.w_b[2]]);
        let qdot = [0.5 * qd[0], 0.5 * qd[1], 0.5 * qd[2], 0.5 * qd[3]];

        let st = &mut self.state;
        st.q = qnormalize(add4(st.q, qdot, dt));
        st.w_b = add(st.w_b, scale(w_dot, dt));
        self.hist_v.ab3(&mut st.p_ned, st.v_ned, dt);
        self.hist_a.ab2(&mut st.v_ned, a_ned, dt);
        self.spec_force = spec;
        self.time += dt;
    }

    pub fn snapshot(&self) -> FdmState {
        build_fdm_state(
            self.time,
            self.state.p_ned,
            self.state.v_ned,
            self.state.w_b,
            self.spec_force,
            self.state.q,
        )
    }

    pub fn state(&self) -> &State {
        &self.state
    }

    pub fn alt(&self) -> f64 {
        -self.state.p_ned[2]
    }

    pub fn time(&self) -> f64 {
        self.time
    }
}

fn add4(q: Quat, qd: Quat, dt: f64) -> Quat {
    [
        q[0] + qd[0] * dt,
        q[1] + qd[1] * dt,
        q[2] + qd[2] * dt,
        q[3] + qd[3] * dt,
    ]
}
