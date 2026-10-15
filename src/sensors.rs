use crate::atmosphere::isa;
use crate::controller::FdmState;
use crate::math::*;
use std::f64::consts::PI;

pub const DEG2RAD: f64 = PI / 180.0;
pub const RAD2DEG: f64 = 180.0 / PI;
const RE: f64 = 6_371_000.0;

/// Deterministic xorshift64* PRNG (no external deps).
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(if seed == 0 { 0x9E3779B97F4A7C15 } else { seed })
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Standard normal via Box-Muller.
    pub fn gauss(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// A single scalar measurement channel: white noise + constant bias + random-walk drift.
#[derive(Clone, Copy)]
pub struct Channel {
    pub sigma: f64,
    pub bias: f64,
    pub drift: f64,
    walk: f64,
}

impl Channel {
    pub const ZERO: Channel = Channel { sigma: 0.0, bias: 0.0, drift: 0.0, walk: 0.0 };
    pub fn new(sigma: f64, bias: f64, drift: f64) -> Channel {
        Channel { sigma, bias, drift, walk: 0.0 }
    }
    pub fn reset(&mut self) {
        self.walk = 0.0;
    }
    pub fn measure(&mut self, dt: f64, truth: f64, rng: &mut Rng) -> f64 {
        if self.drift != 0.0 {
            self.walk += self.drift * rng.gauss() * dt.sqrt();
        }
        let mut y = truth + self.bias + self.walk;
        if self.sigma > 0.0 {
            y += self.sigma * rng.gauss();
        }
        y
    }
}

/// Geodetic reference for the local NED frame origin.
#[derive(Clone, Copy)]
pub struct GeoOrigin {
    pub lat_rad: f64,
    pub lon_rad: f64,
    pub alt_msl: f64,
}

impl GeoOrigin {
    pub fn new(lat_deg: f64, lon_deg: f64, alt_m: f64) -> GeoOrigin {
        GeoOrigin { lat_rad: lat_deg * DEG2RAD, lon_rad: lon_deg * DEG2RAD, alt_msl: alt_m }
    }
    /// Flat-earth local NED (n, e, d) -> (lat_rad, lon_rad, alt_msl_m).
    pub fn to_geodetic(&self, n: f64, e: f64, d: f64) -> (f64, f64, f64) {
        let lat = self.lat_rad + n / RE;
        let lon = self.lon_rad + e / (RE * self.lat_rad.cos());
        (lat, lon, self.alt_msl - d)
    }
}

#[derive(Clone, Copy)]
pub struct ImuSample {
    pub accel: V3,
    pub gyro: V3,
    pub temp_c: f64,
}

pub struct Imu {
    pub accel: [Channel; 3],
    pub gyro: [Channel; 3],
    pub temp: Channel,
    rng: Rng,
}

impl Imu {
    pub fn new() -> Imu {
        Imu {
            accel: [Channel::ZERO; 3],
            gyro: [Channel::ZERO; 3],
            temp: Channel::ZERO,
            rng: Rng::new(0x1),
        }
    }
    pub fn reset(&mut self) {
        for c in self.accel.iter_mut().chain(self.gyro.iter_mut()) {
            c.reset();
        }
        self.temp.reset();
    }
    pub fn sample(&mut self, dt: f64, s: &FdmState) -> ImuSample {
        let mut accel = [0.0; 3];
        let mut gyro = [0.0; 3];
        for i in 0..3 {
            accel[i] = self.accel[i].measure(dt, s.accel_body[i], &mut self.rng);
            gyro[i] = self.gyro[i].measure(dt, s.omega[i], &mut self.rng);
        }
        let temp = self.temp.measure(dt, isa(s.alt).t - 273.15, &mut self.rng);
        ImuSample { accel, gyro, temp_c: temp }
    }
}

#[derive(Clone, Copy)]
pub struct BaroSample {
    pub pressure_pa: f64,
    pub alt_m: f64,
    pub temp_c: f64,
}

pub struct Baro {
    pub pres: Channel,
    pub temp: Channel,
    rng: Rng,
}

impl Baro {
    pub fn new() -> Baro {
        Baro { pres: Channel::ZERO, temp: Channel::ZERO, rng: Rng::new(0x2) }
    }
    pub fn reset(&mut self) {
        self.pres.reset();
        self.temp.reset();
    }
    pub fn sample(&mut self, dt: f64, s: &FdmState) -> BaroSample {
        let atm = isa(s.alt);
        let p = self.pres.measure(dt, atm.p, &mut self.rng);
        let t = self.temp.measure(dt, atm.t - 273.15, &mut self.rng);
        BaroSample { pressure_pa: p, alt_m: s.alt, temp_c: t }
    }
}

#[derive(Clone, Copy)]
pub struct MagSample {
    pub field_gauss: V3,
}

pub struct Mag {
    pub field_ned: V3,
    pub ch: [Channel; 3],
    rng: Rng,
}

impl Mag {
    pub fn new() -> Mag {
        Mag {
            field_ned: [0.22, 0.0, 0.44],
            ch: [Channel::ZERO; 3],
            rng: Rng::new(0x3),
        }
    }
    pub fn reset(&mut self) {
        for c in self.ch.iter_mut() {
            c.reset();
        }
    }
    pub fn sample(&mut self, dt: f64, s: &FdmState) -> MagSample {
        let body = qrot_inv(s.quat, self.field_ned);
        let mut field = [0.0; 3];
        for i in 0..3 {
            field[i] = self.ch[i].measure(dt, body[i], &mut self.rng);
        }
        MagSample { field_gauss: field }
    }
}

#[derive(Clone, Copy)]
pub struct GpsSample {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    pub vel_ned: V3,
    pub fix_type: u8,
    pub satellites: u8,
    pub eph_m: f64,
    pub epv_m: f64,
    pub cog_deg: f64,
    pub yaw_deg: f64,
}

pub struct Gps {
    pub origin: GeoOrigin,
    pub pos: [Channel; 3],
    pub vel: [Channel; 3],
    pub fix_type: u8,
    pub satellites: u8,
    pub eph_m: f64,
    pub epv_m: f64,
    rng: Rng,
}

impl Gps {
    pub fn new(origin: GeoOrigin) -> Gps {
        Gps {
            origin,
            pos: [Channel::ZERO; 3],
            vel: [Channel::ZERO; 3],
            fix_type: 3,
            satellites: 10,
            eph_m: 0.5,
            epv_m: 0.8,
            rng: Rng::new(0x4),
        }
    }
    pub fn reset(&mut self) {
        for c in self.pos.iter_mut().chain(self.vel.iter_mut()) {
            c.reset();
        }
    }
    pub fn sample(&mut self, dt: f64, s: &FdmState) -> GpsSample {
        let mut p = [0.0; 3];
        let mut v = [0.0; 3];
        for i in 0..3 {
            p[i] = self.pos[i].measure(dt, s.pos_ned[i], &mut self.rng);
            v[i] = self.vel[i].measure(dt, s.vel_ned[i], &mut self.rng);
        }
        let (lat, lon, alt) = self.origin.to_geodetic(p[0], p[1], p[2]);
        let cog = if v[0].hypot(v[1]) > 0.5 {
            let mut c = v[1].atan2(v[0]) * RAD2DEG;
            if c < 0.0 {
                c += 360.0;
            }
            c
        } else {
            0.0
        };
        let mut yaw = s.euler[2] * RAD2DEG;
        yaw = yaw.rem_euclid(360.0);
        GpsSample {
            lat_deg: lat * RAD2DEG,
            lon_deg: lon * RAD2DEG,
            alt_m: alt,
            vel_ned: v,
            fix_type: self.fix_type,
            satellites: self.satellites,
            eph_m: self.eph_m,
            epv_m: self.epv_m,
            cog_deg: cog,
            yaw_deg: yaw,
        }
    }
}

pub struct SensorFrame {
    pub time_usec: u64,
    pub imu: ImuSample,
    pub baro: BaroSample,
    pub mag: MagSample,
    pub gps: GpsSample,
}

pub struct SensorSuite {
    pub imu: Imu,
    pub baro: Baro,
    pub mag: Mag,
    pub gps: Gps,
}

impl SensorSuite {
    /// Deterministic, noise-free suite (all channels zero).
    pub fn new(origin: GeoOrigin) -> SensorSuite {
        SensorSuite {
            imu: Imu::new(),
            baro: Baro::new(),
            mag: Mag::new(),
            gps: Gps::new(origin),
        }
    }

    /// MEMS-class noise (white + bias + random walk) on every channel.
    pub fn with_noise(origin: GeoOrigin) -> SensorSuite {
        let mut s = SensorSuite::new(origin);
        s.imu.accel = [Channel::new(0.02, 0.05, 0.01); 3];
        s.imu.gyro = [Channel::new(0.002, 0.001, 0.0005); 3];
        s.imu.temp = Channel::new(0.1, 0.0, 0.0);
        s.baro.pres = Channel::new(10.0, 0.0, 1.0);
        s.baro.temp = Channel::new(0.1, 0.0, 0.0);
        s.mag.ch = [Channel::new(0.002, 0.0, 0.0002); 3];
        s.gps.pos = [Channel::new(0.5, 0.0, 0.1); 3];
        s.gps.vel = [Channel::new(0.1, 0.0, 0.02); 3];
        s
    }
    pub fn reset(&mut self) {
        self.imu.reset();
        self.baro.reset();
        self.mag.reset();
        self.gps.reset();
    }
    pub fn sample(&mut self, dt: f64, s: &FdmState) -> SensorFrame {
        SensorFrame {
            time_usec: (s.time * 1e6) as u64,
            imu: self.imu.sample(dt, s),
            baro: self.baro.sample(dt, s),
            mag: self.mag.sample(dt, s),
            gps: self.gps.sample(dt, s),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn level_state() -> FdmState {
        FdmState {
            time: 0.0,
            pos_ned: [0.0, 0.0, -50.0],
            vel_ned: [0.0; 3],
            vel_body: [0.0; 3],
            omega: [0.0; 3],
            accel_body: [0.0, 0.0, -9.80665],
            quat: [1.0, 0.0, 0.0, 0.0],
            euler: [0.0; 3],
            alt: 50.0,
        }
    }

    #[test]
    fn zero_noise_matches_truth() {
        let mut s = SensorSuite::new(GeoOrigin::new(47.0, 8.0, 500.0));
        let st = level_state();
        let f = s.sample(0.004, &st);
        assert_eq!(f.imu.accel[2], -9.80665);
        assert_eq!(f.imu.gyro[0], 0.0);
        assert_eq!(f.baro.alt_m, 50.0);
    }

    #[test]
    fn noise_perturbs_and_is_deterministic() {
        let st = level_state();
        let mut a = SensorSuite::with_noise(GeoOrigin::new(47.0, 8.0, 500.0));
        let mut b = SensorSuite::with_noise(GeoOrigin::new(47.0, 8.0, 500.0));
        let mut max_dev = 0.0_f64;
        for _ in 0..1000 {
            let fa = a.sample(0.004, &st);
            let fb = b.sample(0.004, &st);
            assert_eq!(fa.imu.accel, fb.imu.accel);
            max_dev = max_dev.max((fa.imu.accel[2] + 9.80665).abs());
        }
        assert!(max_dev > 0.01, "expected accel noise, got {max_dev}");
    }
}
