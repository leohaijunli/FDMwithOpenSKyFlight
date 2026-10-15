import { IDX } from './rfdm.js';
import * as Mav from './Mavlink.js';

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const RE = 6371000;

/** International Standard Atmosphere (troposphere). */
function isa(altM) {
  const T0 = 288.15;
  const L = 0.0065;
  const P0 = 101325;
  const g = 9.80665;
  const R = 287.05287;
  const h = Math.max(0, altM);
  const t = T0 - L * h;
  const p = P0 * Math.pow(t / T0, g / (R * L));
  return { t, p };
}

/** Rotate vector `v` by quaternion `q` = [w, x, y, z]. */
function rotate(q, v) {
  const w = q[0];
  const x = q[1];
  const y = q[2];
  const z = q[3];
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ];
}

function conj(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

function toGeodetic(origin, n, e, d) {
  const lat = origin.lat + n / RE;
  const lon = origin.lon + e / (RE * Math.cos(origin.lat));
  return [lat, lon, origin.alt - d];
}

/**
 * Bridges the browser FDM to PX4 SITL through the dev bridge: streams HIL sensor
 * frames and applies the actuator commands PX4 returns. The plant is stepped
 * with motors (see `Rfdm.stepMotors`), not the local stabilizer.
 */
export default class Px4Link {
  constructor() {
    this.ws = null;
    this.motors = [0, 0, 0, 0];
    this.px4Connected = false;
    this.framesSent = 0;
    this.actuatorsReceived = 0;
    this._seq = 0;
    this._parser = new Mav.MavParser();
    this._origin = null;
    this._lastHb = -1.0;
    this.rc = new Array(12).fill(1500);
    this.rssi = 254;
  }

  /** Feed RC stick PWMs (µs) to PX4; PX4 default mapping: 1=roll, 2=pitch, 3=throttle, 4=yaw. */
  setRcChannels(chan) {
    for (let i = 0; i < 12; i++) this.rc[i] = Math.max(800, Math.min(2200, chan[i] ?? 1500));
  }

  setOrigin(latDeg, lonDeg, altM) {
    this._origin = { lat: latDeg * DEG2RAD, lon: lonDeg * DEG2RAD, alt: altM };
  }

  connect(url) {
    this.disconnect();
    if (typeof WebSocket === 'undefined') return;
    try {
      this.ws = new WebSocket(url);
    } catch {
      this.ws = null;
      return;
    }
    this.ws.binaryType = 'arraybuffer';
    this.ws.onmessage = (ev) => this._onMessage(ev.data);
    this.ws.onclose = () => {
      this.px4Connected = false;
      this.ws = null;
    };
    this.ws.onerror = () => {};
  }

  disconnect() {
    if (this.ws) {
      try {
        this.ws.close();
      } catch {
        /* ignore */
      }
      this.ws = null;
    }
    this.px4Connected = false;
  }

  get active() {
    return !!this.ws && this.ws.readyState === WebSocket.OPEN;
  }

  _onMessage(data) {
    if (typeof data === 'string') {
      this.px4Connected = data.includes('1');
      return;
    }
    this._parser.push(new Uint8Array(data));
    let m;
    while ((m = this._parser.next())) {
      if (m.id === Mav.MSG.HIL_ACTUATOR_CONTROLS) {
        const c = Mav.decodeHilActuatorControls(m.payload);
        if (c) {
          for (let i = 0; i < 4; i++) {
            this.motors[i] = Math.max(0, Math.min(1, c.controls[i]));
          }
          this.actuatorsReceived++;
        }
      }
    }
  }

  _nextSeq() {
    const s = this._seq;
    this._seq = (this._seq + 1) & 0xff;
    return s;
  }

  /** Stream HIL frames derived from the current plant state. */
  sendSensors(st) {
    if (!this.active || !this.px4Connected) return;

    const timeUsec = st[IDX.time] * 1e6;
    if (st[IDX.time] - this._lastHb >= 1.0) {
      this._send(Mav.encodeHeartbeat(this._nextSeq()));
      this._lastHb = st[IDX.time];
    }
    const quat = [st[IDX.qw], st[IDX.qx], st[IDX.qy], st[IDX.qz]];
    const accel = [st[IDX.ax], st[IDX.ay], st[IDX.az]];
    const omega = [st[IDX.p], st[IDX.q], st[IDX.r]];
    const vel = [st[IDX.velN], st[IDX.velE], st[IDX.velD]];

    const [lat, lon, altMsl] = this._origin
      ? toGeodetic(this._origin, st[IDX.posN], st[IDX.posE], st[IDX.posD])
      : [0, 0, st[IDX.alt]];
    const atm = isa(altMsl);
    const mag = rotate(conj(quat), [0.22, 0, 0.44]);

    const sensor = {
      time_usec: timeUsec,
      accel,
      gyro: omega,
      mag,
      pressure_pa: atm.p,
      alt_m: altMsl,
      temp_c: atm.t - 273.15,
    };
    const cog =
      Math.hypot(vel[0], vel[1]) > 0.5
        ? ((Math.atan2(vel[1], vel[0]) * RAD2DEG) % 360 + 360) % 360
        : 0;
    const yaw = ((st[IDX.yaw] * RAD2DEG) % 360 + 360) % 360;
    const gps = {
      time_usec: timeUsec,
      lat_deg: lat * RAD2DEG,
      lon_deg: lon * RAD2DEG,
      alt_m: altMsl,
      vel_ned: vel,
      fix_type: 3,
      satellites: 10,
      eph_m: 0.5,
      epv_m: 0.8,
      cog_deg: cog,
      yaw_deg: yaw,
    };
    const state = {
      time_usec: timeUsec,
      quat,
      omega,
      lat_deg: lat * RAD2DEG,
      lon_deg: lon * RAD2DEG,
      alt_m: altMsl,
      vel_ned: vel,
      accel_body: accel,
    };

    this._send(Mav.encodeHilSensor(sensor, this._nextSeq()));
    this._send(Mav.encodeHilGps(gps, this._nextSeq()));
    this._send(Mav.encodeHilRcInputsRaw({ time_usec: timeUsec, chan: this.rc, rssi: this.rssi }, this._nextSeq()));
    this._send(Mav.encodeHilStateQuaternion(state, this._nextSeq()));
    this.framesSent++;
  }

  _send(bytes) {
    try {
      this.ws.send(bytes);
    } catch {
      /* ignore */
    }
  }
}
