import { CONFIG } from '../utils/config.js';

const DEADZONE = 0.12;

function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

function deadzone(v) {
  const a = Math.abs(v);
  if (a < DEADZONE) return 0;
  return (v - Math.sign(v) * DEADZONE) / (1 - DEADZONE);
}

/**
 * Produces normalized flight sticks `{ roll, pitch, yaw, vertical }` in [-1, 1]
 * from the device selected by `CONFIG.inputMode`. Yaw and vertical always also
 * respond to the keyboard so those axes remain usable in every mode.
 */
export default class StickInput {
  constructor(domElement) {
    this.domElement = domElement;
    this.keys = {};
    this._mouse = { x: 0.5, y: 0.5 };
    this._pad = { x: 0, y: 0 };
    this._padEl = null;
    this._knobEl = null;

    this._onKeyDown = (e) => {
      const tag = e.target.tagName;
      if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
      this.keys[e.code] = true;
    };
    this._onKeyUp = (e) => {
      const tag = e.target.tagName;
      if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
      this.keys[e.code] = false;
    };
    this._onMouseMove = (e) => {
      this._mouse.x = e.clientX / window.innerWidth;
      this._mouse.y = e.clientY / window.innerHeight;
    };
    this._onBlur = () => {
      this.keys = {};
    };

    document.addEventListener('keydown', this._onKeyDown);
    document.addEventListener('keyup', this._onKeyUp);
    document.addEventListener('mousemove', this._onMouseMove);
    window.addEventListener('blur', this._onBlur);
  }

  /** Attach the on-screen joystick pad/knob elements (used by the 'joystick' mode). */
  setJoystickElements(padEl, knobEl) {
    this._padEl = padEl;
    this._knobEl = knobEl;
    if (!padEl) return;

    let dragging = false;
    const move = (e) => {
      const r = padEl.getBoundingClientRect();
      let dx = (e.clientX - (r.left + r.width / 2)) / (r.width / 2);
      let dy = (e.clientY - (r.top + r.height / 2)) / (r.height / 2);
      const len = Math.hypot(dx, dy);
      if (len > 1) {
        dx /= len;
        dy /= len;
      }
      this._pad.x = dx;
      this._pad.y = -dy;
      this._updateKnob();
    };
    const end = () => {
      dragging = false;
      this._pad.x = 0;
      this._pad.y = 0;
      this._updateKnob();
    };

    padEl.addEventListener('pointerdown', (e) => {
      dragging = true;
      padEl.setPointerCapture(e.pointerId);
      move(e);
    });
    padEl.addEventListener('pointermove', (e) => {
      if (dragging) move(e);
    });
    padEl.addEventListener('pointerup', end);
    padEl.addEventListener('pointercancel', end);
  }

  _updateKnob() {
    if (!this._knobEl || !this._padEl) return;
    const r = this._padEl.clientWidth / 2;
    const kx = this._pad.x * r;
    const ky = -this._pad.y * r;
    this._knobEl.style.transform = `translate(calc(-50% + ${kx}px), calc(-50% + ${ky}px))`;
  }

  /** Keyboard sticks, shared by every mode for the yaw/vertical axes. */
  _keys() {
    const k = this.keys;
    const pitch = (k['KeyW'] || k['ArrowUp'] ? 1 : 0) - (k['KeyS'] || k['ArrowDown'] ? 1 : 0);
    const roll = (k['KeyD'] || k['ArrowRight'] ? 1 : 0) - (k['KeyA'] || k['ArrowLeft'] ? 1 : 0);
    const yaw = (k['KeyE'] ? 1 : 0) - (k['KeyQ'] ? 1 : 0);
    const vertical = (k['Space'] ? 1 : 0) - (k['ShiftLeft'] || k['ShiftRight'] ? 1 : 0);
    return { roll, pitch, yaw, vertical };
  }

  _gamepad() {
    const pads = typeof navigator.getGamepads === 'function' ? navigator.getGamepads() : [];
    const gp = pads && pads[0];
    if (!gp) return this._keys();
    const ax = gp.axes;
    return {
      roll: deadzone(ax[0] || 0),
      pitch: deadzone(-(ax[1] || 0)),
      yaw: deadzone(ax[2] || 0),
      vertical: deadzone(-(ax[3] || 0)),
    };
  }

  _onscreen() {
    const k = this._keys();
    return { roll: this._pad.x, pitch: this._pad.y, yaw: k.yaw, vertical: k.vertical };
  }

  _mouseAbs() {
    const k = this._keys();
    const x = clamp((this._mouse.x - 0.5) * 2, -1, 1);
    const y = clamp((0.5 - this._mouse.y) * 2, -1, 1);
    return { roll: deadzone(x), pitch: deadzone(y), yaw: k.yaw, vertical: k.vertical };
  }

  getSticks() {
    switch (CONFIG.inputMode) {
      case 'gamepad':
        return this._gamepad();
      case 'joystick':
        return this._onscreen();
      case 'mouse':
        return this._mouseAbs();
      default:
        return this._keys();
    }
  }

  dispose() {
    document.removeEventListener('keydown', this._onKeyDown);
    document.removeEventListener('keyup', this._onKeyUp);
    document.removeEventListener('mousemove', this._onMouseMove);
    window.removeEventListener('blur', this._onBlur);
  }
}
