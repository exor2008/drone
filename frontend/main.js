import * as THREE from "three";

import { GUI } from "three/addons/libs/lil-gui.module.min.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import WebGL from "three/addons/capabilities/WebGL.js";

let renderer,
  scene,
  camera,
  controls,
  material,
  cmtextures,
  voxelData,
  texture,
  sizeX,
  sizeY,
  sizeZ,
  gltfObj;

function init() {
  console.log("zbs");
  scene = new THREE.Scene();

  initRenderer();
  initCamera();
  initCameraControl();
  loadModel("gltf/x-wing.glb");
  scene.add(new THREE.AxesHelper(20));

  // initGui();
  // initVoxelData();
  // initTexture();
  // initMaterial();
  // initMesh();

  window.addEventListener("resize", onWindowResize);
  render();
}

function initRenderer() {
  // Create renderer
  renderer = new THREE.WebGLRenderer();
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

  renderer.getContext().enable(renderer.getContext().DEPTH_TEST);
}

function initCamera() {
  camera = new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    1,
    1000
  );
  camera.position.set(20, 0, 0);
  camera.lookAt(new THREE.Vector3(0, 0, 0));
  scene.add(camera);

  return camera;
}

function initCameraControl() {
  // Create controls
  controls = new OrbitControls(camera, renderer.domElement);

  controls.minDistance = 20;
  controls.maxDistance = 50;
  controls.maxPolarAngle = Math.PI / 2;
}

function loadModel(dataURL) {
  const loader = new GLTFLoader();
  loader.load(
    dataURL,
    async function (gltf) {
      const ambientLight = new THREE.AmbientLight(0xffffff); // Ambient light
      scene.add(ambientLight);

      const directionalLight = new THREE.DirectionalLight(0xffffff, 1); // Directional light
      directionalLight.position.set(300, 0, 50).normalize();
      directionalLight.lookAt(100, 100, 50);
      scene.add(directionalLight);

      // gltf.scene.scale.set(10, 10, 10);

      await renderer.compileAsync(gltf.scene, camera, scene);
      scene.add(gltf.scene);
      gltfObj = gltf;
    },
    undefined,
    function (error) {
      console.error("Error loading model:", error);
    }
  );
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize(window.innerWidth, window.innerHeight);
}

function render() {
  renderer.render(scene, camera);
}

function animate() {
  requestAnimationFrame(animate);

  if (gltfObj !== null) {
    const xhr = new XMLHttpRequest();
    xhr.open("GET", "http://127.0.0.1:8000/angle", false);
    // Set up event listeners for completion or error
    xhr.onload = function () {
      if (xhr.status === 200) {
        const resp = JSON.parse(xhr.responseText);
        const quaternion = new THREE.Quaternion(
          resp.d[1],
          resp.d[2],
          resp.d[3],
          resp.d[0]
        );
        gltfObj.scene.setRotationFromQuaternion(quaternion);
      } else {
        console.error("File upload failed");
      }
    };

    xhr.onerror = function () {
      console.error("Network error during file upload");
    };

    xhr.send();
  }

  render();
}

init();
animate();
