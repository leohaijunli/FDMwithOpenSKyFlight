use crate::controller::FdmState;
use crate::sensors::{GeoOrigin, SensorFrame};
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};

const MAVLINK_STX: u8 = 0xFD;
const MAVLINK_STX1: u8 = 0xFE;

pub const MSG_HEARTBEAT: u32 = 0;
pub const MSG_HIL_ACTUATOR_CONTROLS: u32 = 93;
pub const MSG_HIL_SENSOR: u32 = 107;
pub const MSG_HIL_GPS: u32 = 113;
pub const MSG_HIL_STATE_QUATERNION: u32 = 115;

const CRC_HEARTBEAT: u8 = 50;
#[allow(dead_code)]
const CRC_HIL_ACTUATOR_CONTROLS: u8 = 47;
const CRC_HIL_SENSOR: u8 = 108;
const CRC_HIL_GPS: u8 = 124;
const CRC_HIL_STATE_QUATERNION: u8 = 4;

pub const MAV_TYPE_ONBOARD_CONTROLLER: u8 = 2;
pub const MAV_AUTOPILOT_INVALID: u8 = 8;
pub const MAV_STATE_ACTIVE: u8 = 4;

fn crc_accumulate(byte: u8, crc: &mut u16) {
    let tmp = byte ^ (*crc & 0xff) as u8;
    let tmp = tmp ^ (tmp << 4);
    *crc = (*crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
}

fn crc_calc(data: &[u8], extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc_accumulate(b, &mut crc);
    }
    crc_accumulate(extra, &mut crc);
    crc
}

fn pack_v2(msgid: u32, payload: &[u8], crc_extra: u8, seq: u8, sysid: u8, compid: u8) -> Vec<u8> {
    let mut buf = Vec::with_capacity(12 + payload.len());
    buf.push(MAVLINK_STX);
    buf.push(payload.len() as u8);
    buf.push(0);
    buf.push(0);
    buf.push(seq);
    buf.push(sysid);
    buf.push(compid);
    buf.push((msgid & 0xFF) as u8);
    buf.push(((msgid >> 8) & 0xFF) as u8);
    buf.push(((msgid >> 16) & 0xFF) as u8);
    buf.extend_from_slice(payload);
    let crc = crc_calc(&buf[1..], crc_extra);
    buf.push((crc & 0xFF) as u8);
    buf.push((crc >> 8) as u8);
    buf
}

fn push_u64(v: &mut Vec<u8>, x: u64) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_u32(v: &mut Vec<u8>, x: u32) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_i32(v: &mut Vec<u8>, x: i32) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_u16(v: &mut Vec<u8>, x: u16) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_i16(v: &mut Vec<u8>, x: i16) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_u8(v: &mut Vec<u8>, x: u8) {
    v.push(x);
}
fn push_f32(v: &mut Vec<u8>, x: f64) {
    v.extend_from_slice(&(x as f32).to_le_bytes());
}

fn recharge(x: f64) -> i16 {
    (x * 1000.0 / 9.80665).clamp(i16::MIN as f64, i16::MAX as f64) as i16
}
fn i16sat(x: f64) -> i16 {
    x.clamp(i16::MIN as f64, i16::MAX as f64) as i16
}
fn u16sat(x: f64) -> u16 {
    x.clamp(0.0, u16::MAX as f64) as u16
}

pub fn encode_heartbeat(
    seq: u8,
    sysid: u8,
    compid: u8,
    mav_type: u8,
    autopilot: u8,
    system_status: u8,
) -> Vec<u8> {
    let mut p = Vec::with_capacity(9);
    push_u32(&mut p, 0);
    push_u8(&mut p, mav_type);
    push_u8(&mut p, autopilot);
    push_u8(&mut p, 0);
    push_u8(&mut p, system_status);
    push_u8(&mut p, 3);
    pack_v2(MSG_HEARTBEAT, &p, CRC_HEARTBEAT, seq, sysid, compid)
}

pub fn encode_hil_sensor(f: &SensorFrame, seq: u8, sysid: u8, compid: u8) -> Vec<u8> {
    let mut p = Vec::with_capacity(65);
    push_u64(&mut p, f.time_usec);
    push_f32(&mut p, f.imu.accel[0]);
    push_f32(&mut p, f.imu.accel[1]);
    push_f32(&mut p, f.imu.accel[2]);
    push_f32(&mut p, f.imu.gyro[0]);
    push_f32(&mut p, f.imu.gyro[1]);
    push_f32(&mut p, f.imu.gyro[2]);
    push_f32(&mut p, f.mag.field_gauss[0]);
    push_f32(&mut p, f.mag.field_gauss[1]);
    push_f32(&mut p, f.mag.field_gauss[2]);
    push_f32(&mut p, f.baro.pressure_pa / 100.0);
    push_f32(&mut p, 0.0);
    push_f32(&mut p, f.baro.alt_m);
    push_f32(&mut p, f.imu.temp_c);
    push_u32(&mut p, u32::MAX);
    push_u8(&mut p, 0);
    pack_v2(MSG_HIL_SENSOR, &p, CRC_HIL_SENSOR, seq, sysid, compid)
}

pub fn encode_hil_gps(f: &SensorFrame, seq: u8, sysid: u8, compid: u8) -> Vec<u8> {
    let g = &f.gps;
    let vn = g.vel_ned[0];
    let ve = g.vel_ned[1];
    let vd = g.vel_ned[2];
    let speed = (vn * vn + ve * ve).sqrt();
    let mut p = Vec::with_capacity(39);
    push_u64(&mut p, f.time_usec);
    push_i32(&mut p, (g.lat_deg * 1e7) as i32);
    push_i32(&mut p, (g.lon_deg * 1e7) as i32);
    push_i32(&mut p, (g.alt_m * 1000.0) as i32);
    push_u16(&mut p, u16sat(g.eph_m * 100.0));
    push_u16(&mut p, u16sat(g.epv_m * 100.0));
    push_u16(&mut p, u16sat(speed * 100.0));
    push_i16(&mut p, i16sat(vn * 100.0));
    push_i16(&mut p, i16sat(ve * 100.0));
    push_i16(&mut p, i16sat(vd * 100.0));
    push_u16(&mut p, u16sat(g.cog_deg * 100.0));
    push_u8(&mut p, g.fix_type);
    push_u8(&mut p, g.satellites);
    push_u8(&mut p, 0);
    push_u16(&mut p, u16sat(g.yaw_deg * 100.0));
    pack_v2(MSG_HIL_GPS, &p, CRC_HIL_GPS, seq, sysid, compid)
}

pub fn encode_hil_state_quaternion(
    s: &FdmState,
    origin: &GeoOrigin,
    seq: u8,
    sysid: u8,
    compid: u8,
) -> Vec<u8> {
    let (lat, lon, alt) = origin.to_geodetic(s.pos_ned[0], s.pos_ned[1], s.pos_ned[2]);
    let mut p = Vec::with_capacity(64);
    push_u64(&mut p, (s.time * 1e6) as u64);
    for i in 0..4 {
        push_f32(&mut p, s.quat[i]);
    }
    push_f32(&mut p, s.omega[0]);
    push_f32(&mut p, s.omega[1]);
    push_f32(&mut p, s.omega[2]);
    push_i32(&mut p, (lat * 180.0 / std::f64::consts::PI * 1e7) as i32);
    push_i32(&mut p, (lon * 180.0 / std::f64::consts::PI * 1e7) as i32);
    push_i32(&mut p, (alt * 1000.0) as i32);
    push_i16(&mut p, i16sat(s.vel_ned[0] * 100.0));
    push_i16(&mut p, i16sat(s.vel_ned[1] * 100.0));
    push_i16(&mut p, i16sat(s.vel_ned[2] * 100.0));
    push_u16(&mut p, 0);
    push_u16(&mut p, 0);
    push_i16(&mut p, recharge(s.accel_body[0]));
    push_i16(&mut p, recharge(s.accel_body[1]));
    push_i16(&mut p, recharge(s.accel_body[2]));
    pack_v2(MSG_HIL_STATE_QUATERNION, &p, CRC_HIL_STATE_QUATERNION, seq, sysid, compid)
}

pub struct ActuatorControls {
    pub time_usec: u64,
    pub controls: [f64; 16],
    pub mode: u8,
    pub flags: u64,
}

pub fn decode_hil_actuator_controls(payload: &[u8]) -> Option<ActuatorControls> {
    if payload.len() < 81 {
        return None;
    }
    let time_usec = u64::from_le_bytes(payload[0..8].try_into().ok()?);
    let flags = u64::from_le_bytes(payload[8..16].try_into().ok()?);
    let mut controls = [0.0f64; 16];
    for i in 0..16 {
        let o = 16 + 4 * i;
        controls[i] = f32::from_le_bytes(payload[o..o + 4].try_into().ok()?) as f64;
    }
    let mode = payload[80];
    Some(ActuatorControls { time_usec, controls, mode, flags })
}

/// Incremental MAVLink v1/v2 frame parser.
pub struct MavParser {
    buf: Vec<u8>,
}

impl MavParser {
    pub fn new() -> MavParser {
        MavParser { buf: Vec::new() }
    }
    pub fn push(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
    }
    /// Returns (msgid, payload) for the next complete frame, or None if more bytes are needed.
    pub fn next_message(&mut self) -> Option<(u32, Vec<u8>)> {
        loop {
            let stx = self
                .buf
                .iter()
                .position(|&b| b == MAVLINK_STX || b == MAVLINK_STX1)?;
            if stx > 0 {
                self.buf.drain(0..stx);
            }
            if self.buf[0] == MAVLINK_STX1 {
                if self.buf.len() < 6 {
                    return None;
                }
                let len = self.buf[1] as usize;
                let total = 8 + len;
                if self.buf.len() < total {
                    return None;
                }
                let msgid = self.buf[5] as u32;
                let payload = self.buf[6..6 + len].to_vec();
                self.buf.drain(0..total);
                return Some((msgid, payload));
            }
            if self.buf.len() < 10 {
                return None;
            }
            let incompat = self.buf[2];
            let len = self.buf[1] as usize;
            let sig = if incompat & 0x01 != 0 { 13 } else { 0 };
            let total = 12 + len + sig;
            if self.buf.len() < total {
                return None;
            }
            let msgid = (self.buf[7] as u32) | ((self.buf[8] as u32) << 8) | ((self.buf[9] as u32) << 16);
            let payload = self.buf[10..10 + len].to_vec();
            self.buf.drain(0..total);
            return Some((msgid, payload));
        }
    }
}

pub trait Transport {
    fn send(&mut self, data: &[u8]) -> io::Result<()>;
    fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize>;
}

/// PX4 connects to this TCP server (default port 4560) and exchanges Simulator MAVLink.
pub struct TcpServerTransport {
    listener: TcpListener,
    stream: Option<TcpStream>,
}

impl TcpServerTransport {
    pub fn bind(addr: &str) -> io::Result<TcpServerTransport> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        Ok(TcpServerTransport { listener, stream: None })
    }
    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }
    fn ensure_connected(&mut self) -> io::Result<()> {
        if self.stream.is_none() {
            match self.listener.accept() {
                Ok((s, _)) => {
                    s.set_nonblocking(true)?;
                    self.stream = Some(s);
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {}
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }
}

impl Transport for TcpServerTransport {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        self.ensure_connected()?;
        if let Some(s) = self.stream.as_mut() {
            s.write_all(data)?;
        }
        Ok(())
    }
    fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.ensure_connected()?;
        match self.stream.as_mut() {
            Some(s) => s.read(buf),
            None => Err(io::Error::from(io::ErrorKind::WouldBlock)),
        }
    }
}

/// Simulator sends to `remote` and reads replies; used when PX4 runs a UDP endpoint.
pub struct UdpTransport {
    sock: UdpSocket,
    remote: SocketAddr,
}

impl UdpTransport {
    pub fn new(local: &str, remote: &str) -> io::Result<UdpTransport> {
        let sock = UdpSocket::bind(local)?;
        sock.set_nonblocking(true)?;
        let remote: SocketAddr = remote
            .parse()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        Ok(UdpTransport { sock, remote })
    }
}

impl Transport for UdpTransport {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        self.sock.send_to(data, self.remote)?;
        Ok(())
    }
    fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.sock.recv(buf)
    }
}

/// Simulator-side PX4 SITL endpoint: emits HIL sensor frames, ingests actuator outputs.
pub struct SitlLink<T: Transport> {
    pub transport: T,
    pub sysid: u8,
    pub compid: u8,
    seq: u8,
    parser: MavParser,
    motors: [f64; 4],
    pending: bool,
    pub send_state_quaternion: bool,
}

impl<T: Transport> SitlLink<T> {
    pub fn new(transport: T) -> SitlLink<T> {
        SitlLink {
            transport,
            sysid: 1,
            compid: 1,
            seq: 0,
            parser: MavParser::new(),
            motors: [0.0; 4],
            pending: false,
            send_state_quaternion: true,
        }
    }

    fn next_seq(&mut self) -> u8 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    pub fn send_heartbeat(&mut self) -> io::Result<()> {
        let s = self.next_seq();
        let msg = encode_heartbeat(
            s,
            self.sysid,
            self.compid,
            MAV_TYPE_ONBOARD_CONTROLLER,
            MAV_AUTOPILOT_INVALID,
            MAV_STATE_ACTIVE,
        );
        self.transport.send(&msg)
    }

    /// Emit HIL_SENSOR (+ HIL_GPS, + HIL_STATE_QUATERNION) for one simulation step.
    pub fn send_sensors(
        &mut self,
        frame: &SensorFrame,
        state: &FdmState,
        origin: &GeoOrigin,
    ) -> io::Result<()> {
        let (s, sysid, compid) = (self.next_seq(), self.sysid, self.compid);
        let sensor = encode_hil_sensor(frame, s, sysid, compid);
        self.transport.send(&sensor)?;
        let (s, sysid, compid) = (self.next_seq(), self.sysid, self.compid);
        let gps = encode_hil_gps(frame, s, sysid, compid);
        self.transport.send(&gps)?;
        if self.send_state_quaternion {
            let (s, sysid, compid) = (self.next_seq(), self.sysid, self.compid);
            let st = encode_hil_state_quaternion(state, origin, s, sysid, compid);
            self.transport.send(&st)?;
        }
        Ok(())
    }

    /// Drain incoming bytes; returns true if a fresh actuator command was decoded.
    pub fn poll_actuators(&mut self) -> io::Result<bool> {
        let mut buf = [0u8; 4096];
        loop {
            match self.transport.recv(&mut buf) {
                Ok(0) => break,
                Ok(n) => self.parser.push(&buf[..n]),
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }
        let mut got = false;
        while let Some((id, payload)) = self.parser.next_message() {
            if id == MSG_HIL_ACTUATOR_CONTROLS {
                if let Some(c) = decode_hil_actuator_controls(&payload) {
                    for i in 0..4 {
                        self.motors[i] = c.controls[i];
                    }
                    self.pending = true;
                    got = true;
                }
            }
        }
        Ok(got)
    }

    pub fn motors(&self) -> [f64; 4] {
        self.motors
    }
    pub fn has_pending(&self) -> bool {
        self.pending
    }
    pub fn take_motors(&mut self) -> Option<[f64; 4]> {
        if self.pending {
            self.pending = false;
            Some(self.motors)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::f450;
    use crate::fdm::{Fdm, State};
    use crate::math::q_from_euler;
    use crate::sensors::{GeoOrigin, SensorSuite};

    struct MockPx4 {
        sent: Vec<u8>,
        inbox: Vec<u8>,
    }

    impl Transport for MockPx4 {
        fn send(&mut self, data: &[u8]) -> io::Result<()> {
            self.sent.extend_from_slice(data);
            Ok(())
        }
        fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if self.inbox.is_empty() {
                return Err(io::Error::from(io::ErrorKind::WouldBlock));
            }
            let n = buf.len().min(self.inbox.len());
            buf[..n].copy_from_slice(&self.inbox[..n]);
            self.inbox.drain(0..n);
            Ok(n)
        }
    }

    fn actuator_frame(controls: [f32; 4]) -> Vec<u8> {
        let mut p = vec![0u8; 81];
        for i in 0..4 {
            p[16 + 4 * i..20 + 4 * i].copy_from_slice(&controls[i].to_le_bytes());
        }
        pack_v2(MSG_HIL_ACTUATOR_CONTROLS, &p, CRC_HIL_ACTUATOR_CONTROLS, 0, 1, 1)
    }

    #[test]
    fn sitl_loop_end_to_end() {
        let origin = GeoOrigin::new(47.397742, 8.545594, 488.0);
        let st = FdmState {
            time: 0.5,
            pos_ned: [0.0, 0.0, -1.0],
            vel_ned: [0.0; 3],
            vel_body: [0.0; 3],
            omega: [0.0; 3],
            accel_body: [0.0, 0.0, -9.80665],
            quat: [1.0, 0.0, 0.0, 0.0],
            euler: [0.0; 3],
            alt: 1.0,
        };
        let mut suite = SensorSuite::with_noise(origin);
        let f = suite.sample(0.004, &st);

        let reply = actuator_frame([0.5, 0.25, 0.125, 0.75]);
        let mut link = SitlLink::new(MockPx4 { sent: Vec::new(), inbox: reply });

        link.send_heartbeat().unwrap();
        link.send_sensors(&f, &st, &origin).unwrap();
        assert!(link.poll_actuators().unwrap());
        let motors = link.take_motors().unwrap();
        assert_eq!(motors, [0.5, 0.25, 0.125, 0.75]);

        // The simulator emitted parseable frames in the documented order.
        let mut p = MavParser::new();
        p.push(&link.transport.sent);
        assert_eq!(p.next_message().unwrap().0, MSG_HEARTBEAT);
        assert_eq!(p.next_message().unwrap().0, MSG_HIL_SENSOR);
        assert_eq!(p.next_message().unwrap().0, MSG_HIL_GPS);
        assert_eq!(p.next_message().unwrap().0, MSG_HIL_STATE_QUATERNION);

        // And the decoded motors drive the plant forward without panicking.
        let state = State {
            p_ned: [0.0, 0.0, 0.0],
            v_ned: [0.0; 3],
            w_b: [0.0; 3],
            q: q_from_euler(0.0, 0.0, 0.0),
        };
        let mut fdm = Fdm::new(f450(), state);
        fdm.step(0.004, &motors);
        assert!(fdm.snapshot().time > 0.0);
    }

    fn frame() -> SensorFrame {
        let origin = GeoOrigin::new(47.397742, 8.545594, 488.0);
        let mut s = SensorSuite::new(origin);
        let st = FdmState {
            time: 1.0,
            pos_ned: [1.0, 2.0, -3.0],
            vel_ned: [1.0, 0.5, -0.25],
            vel_body: [1.0, 0.5, -0.25],
            omega: [0.01, 0.02, 0.03],
            accel_body: [0.0, 0.0, -9.8],
            quat: [1.0, 0.0, 0.0, 0.0],
            euler: [0.0, 0.0, 0.0],
            alt: 3.0,
        };
        s.sample(0.004, &st)
    }

    #[test]
    fn encoded_lengths() {
        let f = frame();
        assert_eq!(encode_hil_sensor(&f, 0, 1, 1).len(), 12 + 65);
        assert_eq!(encode_hil_gps(&f, 0, 1, 1).len(), 12 + 39);
        let st = FdmState {
            time: 1.0,
            pos_ned: [0.0; 3],
            vel_ned: [0.0; 3],
            vel_body: [0.0; 3],
            omega: [0.0; 3],
            accel_body: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            euler: [0.0; 3],
            alt: 0.0,
        };
        let origin = GeoOrigin::new(47.0, 8.0, 488.0);
        assert_eq!(encode_hil_state_quaternion(&st, &origin, 0, 1, 1).len(), 12 + 64);
    }

    #[test]
    fn decode_actuators_roundtrip() {
        let mut payload = vec![0u8; 81];
        payload[0..8].copy_from_slice(&123u64.to_le_bytes());
        for i in 0..16 {
            let v = 0.1 * i as f32;
            payload[16 + 4 * i..20 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        payload[80] = 5;
        let c = decode_hil_actuator_controls(&payload).unwrap();
        assert_eq!(c.time_usec, 123);
        assert_eq!(c.mode, 5);
        assert!((c.controls[3] - 0.3).abs() < 1e-6);
    }

    #[test]
    fn parser_splits_frames() {
        let f = frame();
        let mut bytes = encode_hil_sensor(&f, 0, 1, 1);
        bytes.extend_from_slice(&encode_hil_gps(&f, 1, 1, 1));
        let mut p = MavParser::new();
        p.push(&bytes);
        let (id0, _) = p.next_message().unwrap();
        let (id1, _) = p.next_message().unwrap();
        assert_eq!(id0, MSG_HIL_SENSOR);
        assert_eq!(id1, MSG_HIL_GPS);
        assert!(p.next_message().is_none());
    }
}
