pub type V3 = [f64; 3];
pub type Quat = [f64; 4];

pub fn v3(x: f64, y: f64, z: f64) -> V3 {
    [x, y, z]
}
pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
pub fn normalize(a: V3) -> V3 {
    let n = norm(a);
    if n > 1e-12 {
        scale(a, 1.0 / n)
    } else {
        a
    }
}

pub fn qmul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}
pub fn qconj(q: Quat) -> Quat {
    [q[0], -q[1], -q[2], -q[3]]
}
pub fn qnormalize(q: Quat) -> Quat {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}
pub fn qrot(q: Quat, v: V3) -> V3 {
    let u = [q[1], q[2], q[3]];
    let t = scale(cross(u, v), 2.0);
    add(add(v, scale(t, q[0])), cross(u, t))
}
pub fn qrot_inv(q: Quat, v: V3) -> V3 {
    qrot(qconj(q), v)
}
pub fn q_from_euler(phi: f64, theta: f64, psi: f64) -> Quat {
    let (cr, sr) = ((phi * 0.5).cos(), (phi * 0.5).sin());
    let (cp, sp) = ((theta * 0.5).cos(), (theta * 0.5).sin());
    let (cy, sy) = ((psi * 0.5).cos(), (psi * 0.5).sin());
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}
pub fn euler_from_q(q: Quat) -> (f64, f64, f64) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let phi = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let theta = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let psi = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    (phi, theta, psi)
}
pub fn wrap_pi(a: f64) -> f64 {
    let mut x = a;
    while x > std::f64::consts::PI {
        x -= 2.0 * std::f64::consts::PI;
    }
    while x < -std::f64::consts::PI {
        x += 2.0 * std::f64::consts::PI;
    }
    x
}
