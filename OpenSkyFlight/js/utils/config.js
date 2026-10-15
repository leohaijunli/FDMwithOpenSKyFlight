/**
 * @typedef {Object} AppConfig
 *
 * Real-world terrain:
 * @property {number}  lat                    - Latitude in decimal degrees (default: 45.8326 — Mont Blanc)
 * @property {number}  lon                    - Longitude in decimal degrees (default: 6.8652)
 * @property {number}  zoom                   - Tile zoom level (default: 15)
 * @property {number}  minZoom                - Minimum zoom for LOD rings (default: 3)
 * @property {number}  maxTotalTiles          - Maximum loaded tiles (default: 1000)
 * @property {number}  lodRingRadius          - LOD ring radius in tiles (default: 8)
 * @property {string}  textureMode             - 'satellite' | 'osm' | 'sar' | 'elevation' (default: 'satellite')
 * @property {boolean} hiResMode              - Hi-res zoom 18 mode (default: false)
 *
 * Camera & flight:
 * @property {number}  cameraSpeed            - Flight speed in units/sec, range [1..4000] (default: 800)
 * @property {string}  cameraMode             - 'chase' | 'cockpit' (default: 'chase')
 * @property {number}  mouseSensitivity       - Mouse look sensitivity (default: 0.002)
 *
 * Rendering:
 * @property {number}  maxPixelRatio          - Maximum device pixel ratio (default: 2)
 *
 * Atmosphere:
 * @property {number}  sunElevation           - Sun elevation angle in degrees, range [0..90] (default: 80)
 * @property {number}  sunAzimuth             - Sun azimuth angle in degrees, range [0..360] (default: 180)
 * @property {number}  skyTurbidity           - Atmospheric turbidity, range [1..10] (default: 2.0)
 * @property {number}  skyRayleigh            - Rayleigh scattering coefficient (default: 1.0)
 * @property {boolean} showClouds             - Show cloud layer (default: false)
 * @property {number}  cloudAltitude          - Cloud layer altitude in meters, range [500..12000] (default: 3500)
 * @property {number}  cloudOpacity           - Cloud opacity (default: 0.7)
 * @property {boolean} fogEnabled             - Enable atmospheric fog (default: true)
 * @property {number}  fogDensity             - Fog density (default: 0.000015)
 *
 * UI:
 * @property {boolean} showHud                - Show HUD overlay (default: true)
 * @property {string}  logLevel               - Log level: 'DEBUG' | 'INFO' | 'WARN' | 'ERROR' (default: 'WARN')
 * @property {boolean} showLogs               - Show log panel (default: false)
 * @property {boolean} showMinimap            - Show minimap (default: true)
 * @property {number}  minimapZoom            - Minimap zoom level, range [3..18] (default: 13)
 */

/** Validation rules: { min, max } for numeric clamping */
const CONFIG_VALIDATION = {
  cameraSpeed: { min: 1, max: 4000 },
  droneSpeed: { min: 1, max: 40 },
  climbRate: { min: 0.5, max: 10 },
  sunElevation: { min: 0, max: 90 },
  sunAzimuth: { min: 0, max: 360 },
  skyTurbidity: { min: 1, max: 10 },
  cloudAltitude: { min: 500, max: 12000 },
  cloudOpacity: { min: 0, max: 1 },
  fogDensity: { min: 0, max: 0.01 },
  minimapZoom: { min: 3, max: 18 },
  lat: { min: -90, max: 90 },
  lon: { min: -180, max: 180 },
};

/** @type {AppConfig} */
const CONFIG = {
  cameraSpeed: 800,
  droneSpeed: 15,
  climbRate: 3.0,
  inputMode: 'keyboard', // 'keyboard' | 'gamepad' | 'joystick' | 'mouse'
  sitl: false,
  px4Url: 'ws://localhost:3001',
  cameraMode: 'chase',
  mouseSensitivity: 0.002,
  maxPixelRatio: 2,
  lat: 48.6469, // Victoria International Airport (Sidney, BC)
  lon: -123.4258,
  zoom: 15,
  textureMode: 'satellite', // 'satellite' | 'osm' | 'sar' | 'elevation'
  minZoom: 3,
  maxTotalTiles: 1000,
  lodRingRadius: 8,
  showHud: true,
  logLevel: 'WARN',
  showLogs: false,
  showMinimap: true,
  minimapZoom: 13,
  sunElevation: 80,
  sunAzimuth: 180,
  skyTurbidity: 2.0,
  skyRayleigh: 1.0,
  showClouds: false,
  cloudAltitude: 3500,
  cloudOpacity: 0.7,
  fogEnabled: true,
  fogDensity: 0.000015,
  hiResMode: true,

  /** Rust FDM plant overrides (see `rfdm_set_config`). Keys omitted keep defaults. */
  fdm: {
    mass: 1.4,
    inertia: [0.019, 0.019, 0.0252],
    motor: { kv: 960, resistance: 0.117, i0: 0.45, vmax: 14.63 },
    prop: { diam: 0.23876, ixx: 6.05e-5, senseMult: -1.0 },
    rotors: [
      { x: 0.1651, y: 0.1651, z: -0.025, sense: 1 },
      { x: -0.1651, y: -0.1651, z: -0.025, sense: 1 },
      { x: 0.1651, y: -0.1651, z: -0.025, sense: -1 },
      { x: -0.1651, y: 0.1651, z: -0.025, sense: -1 },
    ],
    aero: { s: 0.016129, cd: 1.0 },
    ground: {
      k: 3000,
      c: 100,
      muS: 0.8,
      muD: 0.5,
      contacts: [
        [0.1651, 0.1651, 0.1524],
        [-0.1651, 0.1651, 0.1524],
        [0.1651, -0.1651, 0.1524],
        [-0.1651, -0.1651, 0.1524],
      ],
    },
    initial: { alt: 5, psi: 0 },
    stabilizer: { maxRatePq: 4.0, maxRateR: 2.0, kp: 0.09, ki: 0.05, limit: 0.3 },
  },

  /** Browser-side control-loop overrides (see MultirotorController). */
  control: {
    physHz: 250,
    maxSubsteps: 25,
    hoverThrottle: 0.405,
    vzKp: 0.06,
    vzKi: 0.02,
    thrIntMax: 0.25,
    maxRatePq: 4.0,
    attKp: 2.5,
    velKp: 0.05,
    velKi: 0.08,
    velIntMax: 1.5,
    maxTilt: 0.5,
  },
};

const listeners = new Set();

function onChange(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

function update(key, value) {
  // Validate and clamp numeric values
  const rule = CONFIG_VALIDATION[key];
  if (rule && typeof value === 'number') {
    value = Math.max(rule.min, Math.min(rule.max, value));
  }

  if (CONFIG[key] !== value) {
    CONFIG[key] = value;
    for (const fn of listeners) fn(key, value);
  }
}

export { CONFIG, onChange, update };
