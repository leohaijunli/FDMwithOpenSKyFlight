import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import Logger from '../utils/Logger.js';

/** Longest dimension of the rendered model (meters). */
const TARGET_LENGTH = 15;

/** Visual spin-rate cap (rad/s) so fast rotors don't strobe/alias at 30-60 fps. */
const MAX_VISUAL_OMEGA = 80;

/** Local NED -> three.js scene frame (x=east, y=up, z=-north). */
const NED_TO_SCENE = new THREE.Matrix4().set(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1);

/**
 * Calibration from this model's axes to the aerospace body frame
 * (x forward, y right, z down): model front is +Z, up is +Y, left is +X.
 */
const MODEL_TO_BODY = new THREE.Matrix4().set(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1);

/**
 * Renders a multirotor mesh from the plant state. Unlike `AircraftManager` it
 * uses the true attitude quaternion directly (no cosmetic roll/pitch).
 */
export default class MultirotorModel {
  constructor(scene) {
    this.scene = scene;
    this.group = null;
    this.ready = false;
    this._props = [];
  }

  async load(url) {
    const gltf = await new GLTFLoader().loadAsync(url);
    this.loadMesh(gltf.scene, { detectProps: true });
  }

  /** Build the scene group from an arbitrary Object3D (GLTF/OBJ/STL/FBX). */
  loadMesh(mesh, { detectProps = false } = {}) {
    if (this.group) {
      this.scene.remove(this.group);
      this.group = null;
    }
    this.ready = false;
    this._props = [];

    if (detectProps) {
      const byName = {};
      mesh.traverse((o) => {
        if (o.name) byName[o.name] = o;
      });
      this._props = [
        ['PropCounterClockwiseFront_low', 1],
        ['PropCounterClockwiseBack_low', 1],
        ['PropClockwiseFront_low', -1],
        ['PropClockwiseBack_low', -1],
      ]
        .map(([name, dir]) => {
          const node = byName[name];
          return node ? { node, dir, angle: 0, base: node.quaternion.clone() } : null;
        })
        .filter(Boolean);
    }

    const box = new THREE.Box3().setFromObject(mesh);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    const scale = maxDim > 0 ? TARGET_LENGTH / maxDim : 1;

    const center = new THREE.Vector3();
    box.getCenter(center);
    center.multiplyScalar(scale);
    mesh.scale.setScalar(scale);
    mesh.position.copy(center).negate();

    const wrapper = new THREE.Group();
    wrapper.quaternion.setFromRotationMatrix(NED_TO_SCENE.clone().multiply(MODEL_TO_BODY));
    wrapper.add(mesh);

    this.group = new THREE.Group();
    this.group.add(wrapper);
    this.group.traverse((child) => {
      child.frustumCulled = false;
    });

    this.scene.add(this.group);
    this.ready = true;
    Logger.info('Aircraft', `Model loaded — scaled ${scale.toFixed(3)}x (${size.x.toFixed(1)}×${size.y.toFixed(1)}×${size.z.toFixed(1)} → ${TARGET_LENGTH}m)`);
  }

  update(state, dt = 0) {
    if (!this.ready) return;
    this.group.position.copy(state.position);
    this.group.quaternion.copy(state.quaternion);

    if (state.rpm && this._props.length) {
      for (let i = 0; i < this._props.length; i++) {
        const p = this._props[i];
        const omega = state.rpm[i] * (Math.PI / 30) * p.dir;
        const visual = Math.max(-MAX_VISUAL_OMEGA, Math.min(MAX_VISUAL_OMEGA, omega));
        p.angle = (p.angle + visual * dt) % (Math.PI * 2);
        p.node.quaternion.copy(p.base);
        p.node.rotateY(p.angle);
      }
    }
  }

  setVisible(visible) {
    if (this.ready) this.group.visible = visible;
  }
}
