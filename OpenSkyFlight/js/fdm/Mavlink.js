/**
 * Minimal MAVLink v2 codec for the PX4 Simulator (HIL) protocol, mirroring
 * `src/px4.rs`. Encodes HIL_SENSOR / HIL_GPS / HIL_STATE_QUATERNION and decodes
 * HIL_ACTUATOR_CONTROLS.
 */

const STX = 0xfd;
const STX1 = 0xfe;

export const MSG = {
  HEARTBEAT: 0,
  HIL_RC_INPUTS_RAW: 92,
  HIL_ACTUATOR_CONTROLS: 93,
  HIL_SENSOR: 107,
  HIL_GPS: 113,
  HIL_STATE_QUATERNION: 115,
};

const CRC_EXTRA = {
  0: 50,
  92: 54,
  93: 47,
  107: 108,
  113: 124,
  115: 4,
};

function crcAccumulate(byte, crc) {
  let tmp = byte ^ (crc & 0xff);
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
  return crc & 0xffff;
}

function crcCalc(bytes, extra) {
  let crc = 0xffff;
  for (let i = 0; i < bytes.length; i++) crc = crcAccumulate(bytes[i], crc);
  return crcAccumulate(extra, crc);
}

function packV2(msgid, payload, seq, sysid, compid) {
  const buf = new Uint8Array(12 + payload.length);
  buf[0] = STX;
  buf[1] = payload.length;
  buf[2] = 0;
  buf[3] = 0;
  buf[4] = seq & 0xff;
  buf[5] = sysid;
  buf[6] = compid;
  buf[7] = msgid & 0xff;
  buf[8] = (msgid >> 8) & 0xff;
  buf[9] = (msgid >> 16) & 0xff;
  buf.set(payload, 10);
  const crc = crcCalc(buf.subarray(1, 10 + payload.length), CRC_EXTRA[msgid]);
  buf[10 + payload.length] = crc & 0xff;
  buf[11 + payload.length] = (crc >> 8) & 0xff;
  return buf;
}

function payload(size, write) {
  const buf = new ArrayBuffer(size);
  write(new DataView(buf));
  return new Uint8Array(buf);
}

function i16(x) {
  return Math.max(-32768, Math.min(32767, Math.trunc(x)));
}

function u16(x) {
  return Math.max(0, Math.min(65535, Math.trunc(x)));
}

function i32(x) {
  return Math.max(-2147483648, Math.min(2147483647, Math.trunc(x)));
}

function usec(x) {
  return BigInt(Math.trunc(x));
}

function recharge(x) {
  return i16((x * 1000.0) / 9.80665);
}

export function encodeHeartbeat(seq, sysid = 1, compid = 1, mavType = 2, autopilot = 8, systemStatus = 4) {
  const p = payload(9, (dv) => {
    dv.setUint8(0, mavType);
    dv.setUint8(1, autopilot);
    dv.setUint8(2, 0);
    dv.setUint32(3, 0, true);
    dv.setUint8(7, systemStatus);
    dv.setUint8(8, 3);
  });
  return packV2(MSG.HEARTBEAT, p, seq, sysid, compid);
}

export function encodeHilSensor(f, seq, sysid = 1, compid = 1) {
  const p = payload(65, (dv) => {
    dv.setBigUint64(0, usec(f.time_usec), true);
    dv.setFloat32(8, f.accel[0], true);
    dv.setFloat32(12, f.accel[1], true);
    dv.setFloat32(16, f.accel[2], true);
    dv.setFloat32(20, f.gyro[0], true);
    dv.setFloat32(24, f.gyro[1], true);
    dv.setFloat32(28, f.gyro[2], true);
    dv.setFloat32(32, f.mag[0], true);
    dv.setFloat32(36, f.mag[1], true);
    dv.setFloat32(40, f.mag[2], true);
    dv.setFloat32(44, f.pressure_pa / 100.0, true);
    dv.setFloat32(48, 0.0, true);
    dv.setFloat32(52, f.alt_m, true);
    dv.setFloat32(56, f.temp_c, true);
    dv.setUint32(60, 0xffffffff, true);
    dv.setUint8(64, 0);
  });
  return packV2(MSG.HIL_SENSOR, p, seq, sysid, compid);
}

export function encodeHilGps(g, seq, sysid = 1, compid = 1) {
  const vn = g.vel_ned[0];
  const ve = g.vel_ned[1];
  const vd = g.vel_ned[2];
  const speed = Math.hypot(vn, ve);
  const p = payload(39, (dv) => {
    dv.setBigUint64(0, usec(g.time_usec), true);
    dv.setUint8(8, g.fix_type);
    dv.setInt32(9, i32(g.lat_deg * 1e7), true);
    dv.setInt32(13, i32(g.lon_deg * 1e7), true);
    dv.setInt32(17, i32(g.alt_m * 1000), true);
    dv.setUint16(21, u16(g.eph_m * 100), true);
    dv.setUint16(23, u16(g.epv_m * 100), true);
    dv.setUint16(25, u16(speed * 100), true);
    dv.setInt16(27, i16(vn * 100), true);
    dv.setInt16(29, i16(ve * 100), true);
    dv.setInt16(31, i16(vd * 100), true);
    dv.setUint16(33, u16(g.cog_deg * 100), true);
    dv.setUint8(35, g.satellites);
    dv.setUint8(36, 0);
    dv.setUint16(37, u16(g.yaw_deg * 100), true);
  });
  return packV2(MSG.HIL_GPS, p, seq, sysid, compid);
}

export function encodeHilRcInputsRaw(rc, seq, sysid = 1, compid = 1) {
  const p = payload(33, (dv) => {
    dv.setBigUint64(0, usec(rc.time_usec), true);
    for (let i = 0; i < 12; i++) dv.setUint16(8 + 2 * i, rc.chan[i] & 0xffff, true);
    dv.setUint8(32, rc.rssi);
  });
  return packV2(MSG.HIL_RC_INPUTS_RAW, p, seq, sysid, compid);
}

export function encodeHilStateQuaternion(s, seq, sysid = 1, compid = 1) {
  const p = payload(64, (dv) => {
    dv.setBigUint64(0, usec(s.time_usec), true);
    dv.setFloat32(8, s.quat[0], true);
    dv.setFloat32(12, s.quat[1], true);
    dv.setFloat32(16, s.quat[2], true);
    dv.setFloat32(20, s.quat[3], true);
    dv.setFloat32(24, s.omega[0], true);
    dv.setFloat32(28, s.omega[1], true);
    dv.setFloat32(32, s.omega[2], true);
    dv.setInt32(36, i32(s.lat_deg * 1e7), true);
    dv.setInt32(40, i32(s.lon_deg * 1e7), true);
    dv.setInt32(44, i32(s.alt_m * 1000), true);
    dv.setInt16(48, i16(s.vel_ned[0] * 100), true);
    dv.setInt16(50, i16(s.vel_ned[1] * 100), true);
    dv.setInt16(52, i16(s.vel_ned[2] * 100), true);
    dv.setUint16(54, 0, true);
    dv.setUint16(56, 0, true);
    dv.setInt16(58, recharge(s.accel_body[0]), true);
    dv.setInt16(60, recharge(s.accel_body[1]), true);
    dv.setInt16(62, recharge(s.accel_body[2]), true);
  });
  return packV2(MSG.HIL_STATE_QUATERNION, p, seq, sysid, compid);
}

export function decodeHilActuatorControls(p) {
  if (p.length < 81) return null;
  const dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
  const controls = new Array(16);
  for (let i = 0; i < 16; i++) controls[i] = dv.getFloat32(8 + 4 * i, true);
  return {
    time_usec: dv.getBigUint64(0, true),
    flags: dv.getBigUint64(73, true),
    controls,
    mode: dv.getUint8(72),
  };
}

/** Incremental MAVLink v1/v2 frame parser. */
export class MavParser {
  constructor() {
    this._buf = new Uint8Array(0);
  }

  push(data) {
    const merged = new Uint8Array(this._buf.length + data.length);
    merged.set(this._buf, 0);
    merged.set(data, this._buf.length);
    this._buf = merged;
  }

  /** Returns { id, payload } for the next complete frame, or null. */
  next() {
    let start = -1;
    for (let i = 0; i < this._buf.length; i++) {
      if (this._buf[i] === STX || this._buf[i] === STX1) {
        start = i;
        break;
      }
    }
    if (start < 0) {
      this._buf = new Uint8Array(0);
      return null;
    }
    if (start > 0) this._buf = this._buf.subarray(start);

    if (this._buf[0] === STX1) {
      if (this._buf.length < 6) return null;
      const len = this._buf[1];
      const total = 8 + len;
      if (this._buf.length < total) return null;
      const id = this._buf[5];
      const payload = this._buf.slice(6, 6 + len);
      this._buf = this._buf.slice(total);
      return { id, payload };
    }

    if (this._buf.length < 10) return null;
    const incompat = this._buf[2];
    const len = this._buf[1];
    const sig = incompat & 0x01 ? 13 : 0;
    const total = 12 + len + sig;
    if (this._buf.length < total) return null;
    const id = this._buf[7] | (this._buf[8] << 8) | (this._buf[9] << 16);
    const payload = this._buf.slice(10, 10 + len);
    this._buf = this._buf.slice(total);
    return { id, payload };
  }
}
