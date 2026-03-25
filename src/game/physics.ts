import type { SceneSettings } from "@ggez/shared";
import {
  addBroadphaseLayer,
  addObjectLayer,
  castRay,
  CastRayStatus,
  createClosestCastRayCollector,
  createDefaultCastRaySettings,
  createWorld,
  createWorldSettings,
  enableCollision,
  filter as crashFilter,
  registerAll,
  rigidBody,
  updateWorld,
  type RigidBody,
  type World
} from "crashcat";
import { Euler, Quaternion, Vector3 } from "three";

export type PhysicsBody = RigidBody;

export type PhysicsWorld = World & {
  gameLayers: {
    ghostObjectLayer: number;
    movingBroadphaseLayer: number;
    movingObjectLayer: number;
    staticBroadphaseLayer: number;
    staticObjectLayer: number;
  };
};

export type PhysicsRaycastHit = {
  body: PhysicsBody;
  distance: number;
  fraction: number;
  normal: Vector3;
  point: Vector3;
  subShapeId: number;
};

let physicsRegistered = false;

export function ensurePhysicsRuntime() {
  if (physicsRegistered) {
    return;
  }

  registerAll();
  physicsRegistered = true;
}

export function createPhysicsWorld(settings: Pick<SceneSettings, "world">): PhysicsWorld {
  ensurePhysicsRuntime();

  const worldSettings = createWorldSettings();
  const staticBroadphaseLayer = addBroadphaseLayer(worldSettings);
  const movingBroadphaseLayer = addBroadphaseLayer(worldSettings);
  const staticObjectLayer = addObjectLayer(worldSettings, staticBroadphaseLayer);
  const movingObjectLayer = addObjectLayer(worldSettings, movingBroadphaseLayer);
  const ghostObjectLayer = addObjectLayer(worldSettings, movingBroadphaseLayer);

  enableCollision(worldSettings, movingObjectLayer, movingObjectLayer);
  enableCollision(worldSettings, movingObjectLayer, staticObjectLayer);

  worldSettings.gravity = [settings.world.gravity.x, settings.world.gravity.y, settings.world.gravity.z];

  const world = createWorld(worldSettings) as PhysicsWorld;
  world.gameLayers = {
    ghostObjectLayer,
    movingBroadphaseLayer,
    movingObjectLayer,
    staticBroadphaseLayer,
    staticObjectLayer
  };

  return world;
}

export function stepPhysicsWorld(world: PhysicsWorld, deltaSeconds: number) {
  ensurePhysicsRuntime();
  updateWorld(world, undefined, deltaSeconds);
}

export function destroyPhysicsWorld(_world: PhysicsWorld) {}

export function toPhysicsVec3(input: { x: number; y: number; z: number }): [number, number, number] {
  return [input.x, input.y, input.z];
}

export function toPhysicsQuat(input: Quaternion): [number, number, number, number] {
  return [input.x, input.y, input.z, input.w];
}

export function toPhysicsQuatFromEuler(input: { x: number; y: number; z: number }): [number, number, number, number] {
  scratchQuaternion.setFromEuler(scratchEuler.set(input.x, input.y, input.z));
  return [scratchQuaternion.x, scratchQuaternion.y, scratchQuaternion.z, scratchQuaternion.w];
}

export function setVector3FromPhysics(target: Vector3, value: readonly number[]) {
  return target.set(value[0] ?? 0, value[1] ?? 0, value[2] ?? 0);
}

export function setQuaternionFromPhysics(target: Quaternion, value: readonly number[]) {
  return target.set(value[0] ?? 0, value[1] ?? 0, value[2] ?? 0, value[3] ?? 1);
}

export function raycastClosest(options: {
  direction: { x: number; y: number; z: number };
  excludeBody?: PhysicsBody;
  length: number;
  origin: { x: number; y: number; z: number };
  world: PhysicsWorld;
}): PhysicsRaycastHit | undefined {
  const queryFilter = crashFilter.forWorld(options.world);
  const collector = createClosestCastRayCollector();
  const settings = createDefaultCastRaySettings();

  if (options.excludeBody) {
    queryFilter.bodyFilter = (candidate) => candidate.id !== options.excludeBody?.id;
  }

  castRay(
    options.world,
    collector,
    settings,
    toPhysicsVec3(options.origin),
    toPhysicsVec3(options.direction),
    options.length,
    queryFilter
  );

  if (collector.hit.status !== CastRayStatus.COLLIDING) {
    return undefined;
  }

  const body = rigidBody.get(options.world, collector.hit.bodyIdB);

  if (!body) {
    return undefined;
  }

  const distance = collector.hit.fraction * options.length;
  scratchPoint.set(
    options.origin.x + options.direction.x * distance,
    options.origin.y + options.direction.y * distance,
    options.origin.z + options.direction.z * distance
  );
  rigidBody.getSurfaceNormal(
    scratchNormalArray,
    body,
    [scratchPoint.x, scratchPoint.y, scratchPoint.z],
    collector.hit.subShapeId
  );

  return {
    body,
    distance,
    fraction: collector.hit.fraction,
    normal: setVector3FromPhysics(scratchNormal, scratchNormalArray).clone(),
    point: scratchPoint.clone(),
    subShapeId: collector.hit.subShapeId
  };
}

const scratchEuler = new Euler();
const scratchNormal = new Vector3();
const scratchNormalArray: [number, number, number] = [0, 0, 0];
const scratchPoint = new Vector3();
const scratchQuaternion = new Quaternion();
