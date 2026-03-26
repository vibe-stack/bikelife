import bikeModelUrl from "../../assets/bike.glb?url";
import sceneManifestUrl from "./scene.runtime.json?url";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { Box3, Color, DirectionalLight, HemisphereLight, Mesh, Vector3, type Object3D } from "three";
import { raycastClosest, type PhysicsWorld } from "../../game/physics";
import { createPublicRuntimeSceneSource, defineGameScene } from "../../game/runtime-scene-sources";
import { applyShadowFlags, configureDirectionalShadowLight } from "../../game/shadows";

const bikeLoader = new GLTFLoader();
bikeLoader.setMeshoptDecoder(MeshoptDecoder);
const BIKE_SCALE = 1;

export const rampTest = defineGameScene({
  id: "ramp-test",
  async mount({ gotoScene, physicsWorld, player, runtimeScene, scene }) {
    const spawn = runtimeScene.entities.find((entity) => entity.type === "player-spawn");
    const spawnPosition = spawn
      ? new Vector3(spawn.transform.position.x, spawn.transform.position.y, spawn.transform.position.z)
      : player?.object.position.clone() ?? new Vector3();
    const spawnYaw = spawn?.transform.rotation.y ?? 0;
    const forward = new Vector3(-Math.sin(spawnYaw), 0, -Math.cos(spawnYaw));
    const right = new Vector3(-forward.z, 0, forward.x);
    const skyColor = new Color("#8fc8ff");
    scene.background = skyColor;

    if (scene.fog && "color" in scene.fog) {
      scene.fog.color.copy(skyColor);
    }

    applyShadowFlags(runtimeScene.root);

    const skyLight = new HemisphereLight("#d8efff", "#d4c2a6", 1.65);
    const sunLight = new DirectionalLight("#fff3d6", 1.2);
    sunLight.position.set(18, 26, 10);
    configureDirectionalShadowLight(sunLight);
    scene.add(skyLight);
    scene.add(sunLight.target);
    scene.add(sunLight);

    const bikeGltf = await bikeLoader.loadAsync(bikeModelUrl);
    const bike = bikeGltf.scene;
    bike.traverse((child) => {
      const candidate = child as Object3D & {
        castShadow?: boolean;
        frustumCulled?: boolean;
        receiveShadow?: boolean;
      };

      if ("castShadow" in candidate) {
        candidate.castShadow = true;
      }

      if ("receiveShadow" in candidate) {
        candidate.receiveShadow = true;
      }

      if ("frustumCulled" in candidate) {
        candidate.frustumCulled = false;
      }
    });
    bike.scale.setScalar(BIKE_SCALE);
    bike.position.copy(spawnPosition).addScaledVector(forward, 2).addScaledVector(right, 1.2);
    bike.position.y = spawnPosition.y + 0.22;
    bike.rotation.y = spawnYaw;
    scene.add(bike);

    const frontWheel = findWheelObject(bike, ["front_wheel", "front-wheel", "frontwheel"], /front.*wheel|wheel.*front/i);
    const backWheel = findWheelObject(bike, ["back_wheel", "back-wheel", "backwheel"], /back.*wheel|wheel.*back/i);
    const wheelCandidates = Array.from(new Set([frontWheel, backWheel].filter(Boolean))) as Mesh[];
    const wheels = wheelCandidates.length >= 2 ? wheelCandidates : [];
    const wheelRadii = wheels.map((wheel) => centerWheelMesh(wheel));
    const bikeGroundOffset = resolveBikeGroundOffset(bike);
    const resolvedWheelRadius =
      wheelRadii.length > 0
        ? wheelRadii.reduce((sum, radius) => sum + radius, 0) / wheelRadii.length
        : 0.34;
    bike.position.y = resolveGroundHeight(
      physicsWorld,
      bike.position.x,
      bike.position.z,
      spawnPosition.y + 12,
      spawnPosition.y
    ) + bikeGroundOffset + 0.01;

    player?.attachBike({
      frontWheelAnchor: frontWheel
        ? {
            x: frontWheel.position.x,
            y: frontWheel.position.y,
            z: frontWheel.position.z
          }
        : undefined,
      groundOffset: bikeGroundOffset,
      mountRadius: 2.4,
      object: bike,
      rearWheelAnchor: backWheel
        ? {
            x: backWheel.position.x,
            y: backWheel.position.y,
            z: backWheel.position.z
          }
        : undefined,
      wheelRadius: resolvedWheelRadius,
      wheels
    });

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.code === "Digit2") {
        void gotoScene("arena");
      }
    };

    window.addEventListener("keydown", handleKeyDown);

    return {
      dispose() {
        player?.attachBike(null);
        scene.remove(skyLight);
        scene.remove(sunLight);
        scene.remove(sunLight.target);
        scene.remove(bike);
        window.removeEventListener("keydown", handleKeyDown);
      }
    };
  },
  source: createPublicRuntimeSceneSource(sceneManifestUrl),
  title: "Main"
});

function centerWheelMesh(mesh: Mesh) {
  mesh.geometry = mesh.geometry.clone();
  mesh.geometry.computeBoundingBox();
  const bounds = mesh.geometry.boundingBox;

  if (!bounds) {
    return 0.34;
  }

  const center = bounds.getCenter(new Vector3());
  const size = bounds.getSize(new Vector3());
  mesh.geometry.translate(-center.x, -center.y, -center.z);
  mesh.position.copy(center.multiply(mesh.scale).applyQuaternion(mesh.quaternion));
  return Math.max(size.y, size.z) * 0.5;
}

function findWheelObject(root: Object3D, exactNames: string[], pattern: RegExp) {
  for (const name of exactNames) {
    const direct = root.getObjectByName(name);

    if (direct instanceof Mesh) {
      return direct;
    }
  }

  let matched: Mesh | null = null;
  root.traverse((child) => {
    if (matched || !(child instanceof Mesh)) {
      return;
    }

    if (pattern.test(child.name)) {
      matched = child;
    }
  });

  return matched;
}

function resolveGroundHeight(world: PhysicsWorld, x: number, z: number, startY: number, fallbackY: number) {
  const hit = raycastClosest({
    direction: { x: 0, y: -1, z: 0 },
    length: 20,
    origin: { x, y: startY, z },
    world
  });
  return hit ? hit.point.y : fallbackY;
}

function resolveBikeGroundOffset(bike: Object3D) {
  const previousPosition = bike.position.clone();
  bike.position.set(0, 0, 0);
  bike.updateMatrixWorld(true);
  const bounds = new Box3().setFromObject(bike);
  bike.position.copy(previousPosition);
  bike.updateMatrixWorld(true);
  return -bounds.min.y;
}
