/**
 * Three.js visualizer for BLSM robot tape playback.
 * Shows robot pose with 3 towers and rotating central body.
 */

class BlsmVisualizer {
  constructor(container) {
    this.container = container;
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.robotBody = null;
    this.towers = [];
    this.cables = [];

    // Robot geometry (from blsm_config.py, scaled to scene units)
    this.SCALE = 0.01; // mm to scene units
    this.TOWER_POSITIONS = [
      { x: 49.1, y: 0, z: 0 },        // Tower 1 (front)
      { x: -24.55, y: 42.5, z: 0 },   // Tower 2 (left rear)
      { x: -24.55, y: -42.5, z: 0 },  // Tower 3 (right rear)
    ];
    this.BASE_RADIUS = 49.1;
    this.REST_HEIGHT = 82.5;
    this.HEIGHT_RANGE = { min: 0, max: 140 };

    // Current pose state
    this.currentPose = { h: 50, yaw: 0, pitch: 0, roll: 0 };

    // Tape data for scrubbing
    this.tapeFrames = [];
    this.tapeDuration = 0;

    // Sync mode
    this.syncMode = false;

    this.init();
  }

  init() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight || 400;

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf0f0f0);

    // Camera
    this.camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);
    this.camera.position.set(3, 2, 3);
    this.camera.lookAt(0, 0.5, 0);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.container.appendChild(this.renderer.domElement);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 5);
    this.scene.add(directionalLight);

    // Grid helper
    const gridHelper = new THREE.GridHelper(4, 20, 0xcccccc, 0xe0e0e0);
    this.scene.add(gridHelper);

    // Axes helper
    const axesHelper = new THREE.AxesHelper(0.5);
    this.scene.add(axesHelper);

    // Create robot geometry
    this.createRobotGeometry();

    // OrbitControls (if available)
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.05;
      this.controls.target.set(0, 0.5, 0);
    }

    // Handle resize
    window.addEventListener('resize', () => this.onResize());

    // Start animation loop
    this.animate();
  }

  createRobotGeometry() {
    const s = this.SCALE;

    // Base platform (circle on ground)
    const baseGeometry = new THREE.CylinderGeometry(
      this.BASE_RADIUS * s,
      this.BASE_RADIUS * s,
      0.02,
      32
    );
    const baseMaterial = new THREE.MeshPhongMaterial({ color: 0x888888 });
    const baseMesh = new THREE.Mesh(baseGeometry, baseMaterial);
    baseMesh.position.y = 0.01;
    this.scene.add(baseMesh);

    // Towers (3 vertical cylinders)
    const towerGeometry = new THREE.CylinderGeometry(0.03, 0.03, 1.5, 16);
    const towerMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 });

    this.TOWER_POSITIONS.forEach((pos, i) => {
      const tower = new THREE.Mesh(towerGeometry, towerMaterial);
      tower.position.set(pos.x * s, 0.75, pos.y * s);
      this.scene.add(tower);
      this.towers.push(tower);
    });

    // Robot body (central sphere/dome that rotates)
    const bodyGroup = new THREE.Group();

    // Main body - soft rounded shape
    const bodyGeometry = new THREE.SphereGeometry(0.35, 32, 24, 0, Math.PI * 2, 0, Math.PI * 0.6);
    const bodyMaterial = new THREE.MeshPhongMaterial({
      color: 0x1abc9c,
      shininess: 30,
    });
    const bodyMesh = new THREE.Mesh(bodyGeometry, bodyMaterial);
    bodyMesh.rotation.x = Math.PI;
    bodyMesh.position.y = 0;
    bodyGroup.add(bodyMesh);

    // Face indicator (small sphere to show orientation)
    const faceGeometry = new THREE.SphereGeometry(0.08, 16, 16);
    const faceMaterial = new THREE.MeshPhongMaterial({ color: 0x2c3e50 });
    const faceMesh = new THREE.Mesh(faceGeometry, faceMaterial);
    faceMesh.position.set(0.25, 0.1, 0);
    bodyGroup.add(faceMesh);

    // Eyes
    const eyeGeometry = new THREE.SphereGeometry(0.04, 12, 12);
    const eyeMaterial = new THREE.MeshPhongMaterial({ color: 0xffffff });
    const eyePupilGeometry = new THREE.SphereGeometry(0.02, 8, 8);
    const eyePupilMaterial = new THREE.MeshPhongMaterial({ color: 0x000000 });

    [-0.08, 0.08].forEach(z => {
      const eye = new THREE.Mesh(eyeGeometry, eyeMaterial);
      eye.position.set(0.32, 0.15, z);
      bodyGroup.add(eye);

      const pupil = new THREE.Mesh(eyePupilGeometry, eyePupilMaterial);
      pupil.position.set(0.35, 0.15, z);
      bodyGroup.add(pupil);
    });

    bodyGroup.position.y = this.REST_HEIGHT * s;
    this.robotBody = bodyGroup;
    this.scene.add(bodyGroup);

    // Cables (lines from towers to body attachment points)
    const cableMaterial = new THREE.LineBasicMaterial({ color: 0x333333 });

    this.TOWER_POSITIONS.forEach((pos, i) => {
      const points = [
        new THREE.Vector3(pos.x * s, 1.4, pos.y * s),
        new THREE.Vector3(pos.x * s * 0.3, this.REST_HEIGHT * s, pos.y * s * 0.3)
      ];
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const cable = new THREE.Line(geometry, cableMaterial);
      this.scene.add(cable);
      this.cables.push(cable);
    });
  }

  updatePose(h, yaw, pitch, roll) {
    this.currentPose = { h, yaw, pitch, roll };

    if (!this.robotBody) return;

    const s = this.SCALE;

    // Map h (0-100 scale) to actual height
    const heightMapped = this.HEIGHT_RANGE.min +
      (h / 100) * (this.HEIGHT_RANGE.max - this.HEIGHT_RANGE.min);
    const height = (this.REST_HEIGHT + heightMapped - 50) * s;

    // Update body position (height)
    this.robotBody.position.y = Math.max(0.2, height);

    // Update body rotation (ZXY order: yaw, pitch, roll)
    // Three.js uses XYZ, so we apply in reverse order
    this.robotBody.rotation.set(0, 0, 0);
    this.robotBody.rotation.y = yaw;      // Yaw around Y
    this.robotBody.rotation.x = pitch;    // Pitch around X
    this.robotBody.rotation.z = roll;     // Roll around Z

    // Update cable endpoints
    this.updateCables();
  }

  updateCables() {
    if (!this.robotBody) return;

    const s = this.SCALE;
    const bodyPos = this.robotBody.position;

    this.TOWER_POSITIONS.forEach((towerPos, i) => {
      if (!this.cables[i]) return;

      // Calculate attachment point on body (rotated with body)
      const attachOffset = new THREE.Vector3(
        towerPos.x * s * 0.3,
        0,
        towerPos.y * s * 0.3
      );
      attachOffset.applyEuler(this.robotBody.rotation);

      const attachPoint = new THREE.Vector3(
        bodyPos.x + attachOffset.x,
        bodyPos.y + attachOffset.y,
        bodyPos.z + attachOffset.z
      );

      // Update cable geometry
      const positions = this.cables[i].geometry.attributes.position;
      positions.array[3] = attachPoint.x;
      positions.array[4] = attachPoint.y;
      positions.array[5] = attachPoint.z;
      positions.needsUpdate = true;
    });
  }

  loadTapeFrames(frames, duration) {
    this.tapeFrames = frames;
    this.tapeDuration = duration;

    // Show first frame
    if (frames.length > 0) {
      const f = frames[0];
      this.updatePose(f.h, f.yaw, f.pitch, f.roll);
    }
  }

  scrubToTime(t) {
    if (this.tapeFrames.length === 0) return;

    // Find frames to interpolate between
    let frameA = this.tapeFrames[0];
    let frameB = this.tapeFrames[0];

    for (let i = 0; i < this.tapeFrames.length - 1; i++) {
      if (this.tapeFrames[i].ts <= t && this.tapeFrames[i + 1].ts >= t) {
        frameA = this.tapeFrames[i];
        frameB = this.tapeFrames[i + 1];
        break;
      }
      if (this.tapeFrames[i].ts > t) break;
      frameA = this.tapeFrames[i];
      frameB = this.tapeFrames[i];
    }

    // Handle edge case: past last frame
    if (t >= this.tapeFrames[this.tapeFrames.length - 1].ts) {
      const lastFrame = this.tapeFrames[this.tapeFrames.length - 1];
      this.updatePose(lastFrame.h, lastFrame.yaw, lastFrame.pitch, lastFrame.roll);
      return;
    }

    // Interpolate
    const dt = frameB.ts - frameA.ts;
    const factor = dt > 0 ? (t - frameA.ts) / dt : 0;

    const h = frameA.h + factor * (frameB.h - frameA.h);
    const yaw = frameA.yaw + factor * (frameB.yaw - frameA.yaw);
    const pitch = frameA.pitch + factor * (frameB.pitch - frameA.pitch);
    const roll = frameA.roll + factor * (frameB.roll - frameA.roll);

    this.updatePose(h, yaw, pitch, roll);
  }

  scrubToProgress(progress) {
    const t = progress * this.tapeDuration;
    this.scrubToTime(t);
  }

  onResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight || 400;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  animate() {
    requestAnimationFrame(() => this.animate());

    if (this.controls) {
      this.controls.update();
    }

    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    if (this.renderer) {
      this.renderer.dispose();
    }
    if (this.controls) {
      this.controls.dispose();
    }
  }
}

// Export for use in page
window.BlsmVisualizer = BlsmVisualizer;
