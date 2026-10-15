import * as THREE from 'three';
import { CONFIG } from '../utils/config.js';
import { IDX } from '../fdm/rfdm.js';
import Px4Link from '../fdm/Px4Link.js';
import StickInput from '../input/StickInput.js';

/** Physics rate of the Rust FDM core (Hz). */
const _ctrl = CONFIG.control ?? {};
const PHYS_HZ = _ctrl.physHz ?? 250;
const DT = 1 / PHYS_HZ;
const MAX_SUBSTEPS = _ctrl.maxSubsteps ?? 25;

/** Throttle stick that holds the F450 in a hover (found empirically). */
const HOVER_THROTTLE = _ctrl.hoverThrottle ?? 0.405;

/** Vertical-velocity hold gains (feed-forward + PI). */
const VZ_KP = _ctrl.vzKp ?? 0.06;
const VZ_KI = _ctrl.vzKi ?? 0.02;
const THR_INT_MAX = _ctrl.thrIntMax ?? 0.25;

/** Max body rate the Rust rate stabilizer is fed (rad/s); must match `stabilizer.rs`. */
const MAX_RATE_PQ = _ctrl.maxRatePq ?? 4.0;
/** Attitude-loop gain: tilt error (rad) -> commanded body rate (rad/s). */
const ATT_KP = _ctrl.attKp ?? 2.5;
/** Horizontal velocity loop: speed error (m/s) -> tilt (rad) + integral. */
const VEL_KP = _ctrl.velKp ?? 0.05;
const VEL_KI = _ctrl.velKi ?? 0.08;
const VEL_INT_MAX = _ctrl.velIntMax ?? 1.5;
/** Maximum commanded tilt (rad) for the velocity loop. */
const MAX_TILT = _ctrl.maxTilt ?? 0.5;

/**
 * Local NED -> three.js scene frame change (x=east, y=up, z=-north).
 * NED is (north, east, down); three.js is Y-up with objects facing -Z.
 */
const NED_TO_SCENE = new THREE.Matrix4().set(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1);

/**
 * Drives the Rust RFDM quadrotor core and exposes the same state interface as
 * `FlightController`, so it can drop into `app.js` unchanged.
 *
 * Attitude is published in the scene frame as `M * R_bn * M^-1`, which equals
 * identity when the aircraft is level and pointing north, keeping the chase
 * camera and Euler readout consistent with the rest of the app.
 */
export default class MultirotorController {
  constructor(camera, domElement, rfdm) {
    this.camera = camera;
    this.domElement = domElement;
    this.rfdm = rfdm;

    this.position = new THREE.Vector3(0, 0.15, 0);
    this.quaternion = new THREE.Quaternion();
    this.yaw = 0;
    this.pitch = 0;
    this.roll = 0;
    this.yawRate = 0;
    this.pitchRate = 0;
    this.rpm = null;

    this.enabled = true;
    /** The FDM supplies true roll, so the app's yaw-rate-derived roll is skipped. */
    this.usesDerivedRoll = false;

    this.input = new StickInput(domElement);
    this.px4 = new Px4Link();
    this.sitl = false;
    this._throttle = HOVER_THROTTLE;
    this._thrInt = 0;
    this._velIntF = 0;
    this._velIntR = 0;
    this._accum = 0;

    this._originY = 0;
    this._originLatched = false;

    this._qM = new THREE.Quaternion().setFromRotationMatrix(NED_TO_SCENE);
    this._qMInv = this._qM.clone().invert();
    this._qBn = new THREE.Quaternion();
    this._euler = new THREE.Euler(0, 0, 0, 'YXZ');
  }

  /** Route control to PX4 SITL (via the WebSocket bridge) or back to local sticks. */
  setSitl(active) {
    this.sitl = !!active;
    if (this.sitl) {
      this.px4.setOrigin(CONFIG.lat, CONFIG.lon, this._originY);
      this.px4.connect(CONFIG.px4Url);
    } else {
      this.px4.disconnect();
    }
  }

  /** Latch the scene-frame ground elevation of the takeoff point (once). */
  setGroundElevation(y) {
    if (!this._originLatched && Number.isFinite(y) && y !== 0) {
      this._originY = y;
      this._originLatched = true;
    }
  }

  setOrientation(yaw, pitch) {
    this.yaw = yaw;
    this.pitch = pitch;
  }

  /** Reset the plant and re-latch the takeoff ground height (used when the terrain location changes). */
  resetOrigin() {
    this.rfdm.reset();
    this._originY = 0;
    this._originLatched = false;
    this._accum = 0;
    this._thrInt = 0;
    this._velIntF = 0;
    this._velIntR = 0;
    this.position.set(0, 5, 0);
    this.quaternion.identity();
    this.yaw = 0;
    this.pitch = 0;
    this.roll = 0;
  }

  _sticks() {
    const cmd = this.input.getSticks();

    const st = this.rfdm.state;
    const psi = st[IDX.yaw];
    const theta = st[IDX.pitch];
    const phi = st[IDX.roll];

    // Ground velocity projected onto the heading axes.
    const cosP = Math.cos(psi);
    const sinP = Math.sin(psi);
    const vFwd = st[IDX.velN] * cosP + st[IDX.velE] * sinP;
    const vRight = -st[IDX.velN] * sinP + st[IDX.velE] * cosP;

    // Velocity PI -> desired tilt; attitude P -> body rate; feed the rate stick.
    const speedMax = CONFIG.droneSpeed;
    const errFwd = cmd.pitch * speedMax - vFwd;
    const errRight = cmd.roll * speedMax - vRight;
    this._velIntF = clamp(this._velIntF + errFwd * DT, -VEL_INT_MAX, VEL_INT_MAX);
    this._velIntR = clamp(this._velIntR + errRight * DT, -VEL_INT_MAX, VEL_INT_MAX);
    const tiltPitch = clamp(-(VEL_KP * errFwd + VEL_KI * this._velIntF), -MAX_TILT, MAX_TILT);
    const tiltRoll = clamp(VEL_KP * errRight + VEL_KI * this._velIntR, -MAX_TILT, MAX_TILT);
    const q = ATT_KP * (tiltPitch - theta);
    const p = ATT_KP * (tiltRoll - phi);
    const rollStick = clamp(p / MAX_RATE_PQ, -1, 1);
    const pitchStick = clamp(q / MAX_RATE_PQ, -1, 1);

    // Vertical velocity hold, with tilt feed-forward to offset lost thrust.
    const vz = -st[IDX.velD];
    const targetVz = cmd.vertical * CONFIG.climbRate;
    const errVz = targetVz - vz;
    this._thrInt = clamp(this._thrInt + VZ_KI * errVz * DT, -THR_INT_MAX, THR_INT_MAX);
    const cosTilt = Math.max(0.5, Math.cos(theta) * Math.cos(phi));
    this._throttle = clamp(HOVER_THROTTLE / cosTilt + VZ_KP * errVz + this._thrInt, 0, 1);

    return { throttle: this._throttle, roll: rollStick, pitch: pitchStick, yaw: cmd.yaw };
  }

  update(dt) {
    if (!this.enabled) return;

    this._accum += Math.min(dt, 0.25);
    let steps = 0;
    while (this._accum >= DT && steps < MAX_SUBSTEPS) {
      const st = this.rfdm.readState();
      if (this.sitl && this.px4.px4Connected) {
        this.px4.sendSensors(st);
        this.rfdm.stepMotors(DT, this.px4.motors);
      } else {
        const s = this._sticks();
        this.rfdm.step(DT, s.throttle, s.roll, s.pitch, s.yaw);
      }
      this._accum -= DT;
      steps++;
    }
    if (steps === MAX_SUBSTEPS) this._accum = 0;

    const st = this.rfdm.readState();

    this.position.set(st[IDX.posE], this._originY - st[IDX.posD], -st[IDX.posN]);

    this._qBn.set(st[IDX.qx], st[IDX.qy], st[IDX.qz], st[IDX.qw]);
    this.quaternion.copy(this._qM).multiply(this._qBn).multiply(this._qMInv);

    this._euler.setFromQuaternion(this.quaternion, 'YXZ');
    this.pitch = this._euler.x;
    this.yaw = this._euler.y;
    this.roll = this._euler.z;
    this.yawRate = st[IDX.r];
    this.pitchRate = st[IDX.q];
    this.rpm = this.rfdm.readRpm();
  }

  dispose() {
    this.input.dispose();
  }
}

function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}
