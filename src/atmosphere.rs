pub const G0: f64 = 9.80665;
pub const RGAS: f64 = 287.05287;
pub const GAMMA: f64 = 1.4;

#[derive(Clone, Copy)]
pub struct Atm {
    pub rho: f64,
    pub p: f64,
    pub t: f64,
    pub a: f64,
}

pub fn isa(alt_m: f64) -> Atm {
    let h = alt_m.max(0.0);
    let (t, p) = if h <= 11000.0 {
        let t = 288.15 - 0.0065 * h;
        (t, 101325.0 * (t / 288.15).powf(-G0 / (-0.0065 * RGAS)))
    } else if h <= 20000.0 {
        let p = 22632.06 * (-(h - 11000.0) * G0 / (RGAS * 216.65)).exp();
        (216.65, p)
    } else if h <= 32000.0 {
        let t = 216.65 + 0.001 * (h - 20000.0);
        (t, 5474.889 * (t / 216.65).powf(-G0 / (0.001 * RGAS)))
    } else {
        let t = 228.65 + 0.0028 * (h - 32000.0);
        (t, 868.0187 * (t / 228.65).powf(-G0 / (0.0028 * RGAS)))
    };
    let rho = p / (RGAS * t);
    let a = (GAMMA * RGAS * t).sqrt();
    Atm { rho, p, t, a }
}
