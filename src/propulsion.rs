use crate::math::*;
use crate::table::interp;
use std::f64::consts::PI;

pub const CT_TABLE: &[(f64, f64)] = &[
    (0.0000, 0.1288),
    (0.0209, 0.1271),
    (0.0522, 0.1252),
    (0.0730, 0.1230),
    (0.1039, 0.1207),
    (0.1252, 0.1181),
    (0.1470, 0.1153),
    (0.1774, 0.1122),
    (0.1983, 0.1089),
    (0.2287, 0.1053),
    (0.2504, 0.1015),
    (0.2774, 0.0975),
    (0.3022, 0.0932),
    (0.3235, 0.0888),
    (0.3539, 0.0842),
    (0.3757, 0.0794),
    (0.4039, 0.0745),
    (0.4274, 0.0695),
    (0.4496, 0.0644),
    (0.4787, 0.0592),
    (0.5004, 0.0538),
    (0.5296, 0.0483),
    (0.5526, 0.0426),
    (0.5783, 0.0369),
    (0.6039, 0.0310),
    (0.6257, 0.0250),
    (0.6548, 0.0189),
    (0.6774, 0.0128),
    (0.7039, 0.0064),
    (0.7291, -0.0001),
];

pub const CP_TABLE: &[(f64, f64)] = &[
    (0.0000, 0.0666),
    (0.0209, 0.0647),
    (0.0522, 0.0628),
    (0.0730, 0.0611),
    (0.1039, 0.0595),
    (0.1252, 0.0581),
    (0.1470, 0.0567),
    (0.1774, 0.0554),
    (0.1983, 0.0542),
    (0.2287, 0.0531),
    (0.2504, 0.0520),
    (0.2774, 0.0509),
    (0.3022, 0.0498),
    (0.3235, 0.0485),
    (0.3539, 0.0471),
    (0.3757, 0.0456),
    (0.4039, 0.0440),
    (0.4274, 0.0422),
    (0.4496, 0.0402),
    (0.4787, 0.0381),
    (0.5004, 0.0358),
    (0.5296, 0.0332),
    (0.5526, 0.0305),
    (0.5783, 0.0275),
    (0.6039, 0.0243),
    (0.6257, 0.0210),
    (0.6548, 0.0175),
    (0.6774, 0.0137),
    (0.7039, 0.0099),
    (0.7291, 0.0061),
];

#[derive(Clone, Copy)]
pub struct Motor {
    pub kv: f64,
    pub resistance: f64,
    pub i0: f64,
    pub vmax: f64,
}

#[derive(Clone, Copy)]
pub struct Prop {
    pub diam: f64,
    pub ixx: f64,
    pub sense_multiplier: f64,
}

#[derive(Clone, Copy)]
pub struct Rotor {
    pub motor: Motor,
    pub prop: Prop,
    pub pos: V3,
    pub sense: f64,
    pub rps: f64,
}

impl Rotor {
    pub fn step(&mut self, dt: f64, throttle: f64, v_b: V3, w_b: V3, rho: f64) -> (V3, V3) {
        let rpm = self.rps * 60.0;
        let volts = self.motor.vmax * throttle;
        let current = (volts - rpm / self.motor.kv) / self.motor.resistance;

        let friction = if rpm > 0.0 {
            self.motor.i0
        } else if rpm < 0.0 {
            -self.motor.i0
        } else {
            current.clamp(-self.motor.i0, self.motor.i0)
        };

        let torque = (current - friction) / self.motor.kv * (60.0 / (2.0 * PI));
        let omega = self.rps * 2.0 * PI;
        let omega_eff = 2.0 * PI * rpm.max(0.0001) / 60.0;
        let engine_power = omega_eff * torque;

        let local = add(v_b, cross(w_b, self.pos));
        let axial = -local[2];

        let d = self.prop.diam;
        let rps = self.rps;
        let j = if rps > 0.01 {
            axial / (d * rps)
        } else {
            axial / d
        };
        let ct = interp(CT_TABLE, j);
        let cp = interp(CP_TABLE, j);
        let thrust = ct * rps * rps * d * d * d * d * rho;

        let local_rps = rps.max(0.01);
        let power_required = cp * local_rps.powi(3) * d.powi(5) * rho;

        let om = if omega > 0.01 { omega } else { 1.0 };
        let excess_torque = (engine_power - power_required) / om;
        let wdot = excess_torque / self.prop.ixx;
        let mut new_rps = rps + wdot / (2.0 * PI) * dt;
        if new_rps < 0.0 {
            new_rps = 0.0;
        }
        self.rps = new_rps;

        let fb = [0.0, 0.0, -thrust];

        let yaw_torque = self.sense * engine_power / omega.max(0.01);
        let hx = self.prop.ixx * omega * self.sense * self.prop.sense_multiplier;
        let vh_body = [0.0, 0.0, -hx];
        let gyro = cross(w_b, vh_body);
        let moment = add(
            cross(self.pos, fb),
            [gyro[0], gyro[1], gyro[2] + yaw_torque],
        );
        (fb, moment)
    }
}

pub struct Quad {
    pub rotors: Vec<Rotor>,
}

impl Quad {
    pub fn step(&mut self, dt: f64, throttle: &[f64], v_b: V3, w_b: V3, rho: f64) -> (V3, V3) {
        let mut force = [0.0; 3];
        let mut moment = [0.0; 3];
        for (i, r) in self.rotors.iter_mut().enumerate() {
            let t = throttle[i].clamp(0.0, 1.0);
            let (f, m) = r.step(dt, t, v_b, w_b, rho);
            force = add(force, f);
            moment = add(moment, m);
        }
        (force, moment)
    }
}

pub fn mix(base: f64, roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    [
        base - roll + pitch + yaw,
        base + roll - pitch + yaw,
        base + roll + pitch - yaw,
        base - roll - pitch - yaw,
    ]
}
