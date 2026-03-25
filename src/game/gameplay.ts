import {
  createMoverSystemDefinition,
  createOpenableSystemDefinition,
  createPathMoverSystemDefinition,
  createScenePathResolver,
  createSequenceSystemDefinition,
  createTriggerSystemDefinition,
  type GameplayRuntimeHost,
  type GameplayRuntimeSystemRegistration
} from "@ggez/gameplay-runtime";
import type { SceneSettings, Transform } from "@ggez/shared";
import type { ThreeRuntimeSceneInstance } from "@ggez/three-runtime";
import { rigidBody } from "crashcat";
import { Euler, Quaternion, type Object3D } from "three";
import type { RuntimePhysicsSession } from "./runtime-physics";

type StarterGameplayHostOptions = {
  runtimePhysics: Pick<RuntimePhysicsSession, "getBody" | "world">;
  runtimeScene: Pick<ThreeRuntimeSceneInstance, "nodesById">;
};

type KinematicPhysicsBody = NonNullable<ReturnType<RuntimePhysicsSession["getBody"]>>;

export function createDefaultGameplaySystems(
  sceneSettings: Pick<SceneSettings, "paths">
): GameplayRuntimeSystemRegistration[] {
  return [
    createTriggerSystemDefinition(),
    createSequenceSystemDefinition(),
    createOpenableSystemDefinition(),
    createMoverSystemDefinition(),
    createPathMoverSystemDefinition(createScenePathResolver(sceneSettings.paths ?? []))
  ];
}

export function mergeGameplaySystems(
  baseSystems: GameplayRuntimeSystemRegistration[],
  sceneSystems: GameplayRuntimeSystemRegistration[]
): GameplayRuntimeSystemRegistration[] {
  const merged = new Map<string, GameplayRuntimeSystemRegistration>();

  baseSystems.forEach((system) => {
    merged.set(system.id, system);
  });
  sceneSystems.forEach((system) => {
    merged.set(system.id, system);
  });

  return Array.from(merged.values());
}

export function createStarterGameplayHost(options: StarterGameplayHostOptions): GameplayRuntimeHost {
  return {
    applyNodeWorldTransform(nodeId, transform) {
      const object = options.runtimeScene.nodesById.get(nodeId);
      const body = options.runtimePhysics.getBody(nodeId);

      if (object) {
        applyTransform(object, transform);
      }

      if (body) {
        applyBodyTransform(options.runtimePhysics.world, body, transform);
      }
    }
  };
}

function applyTransform(object: Object3D, transform: Transform) {
  object.position.set(transform.position.x, transform.position.y, transform.position.z);
  object.rotation.set(transform.rotation.x, transform.rotation.y, transform.rotation.z);
  object.scale.set(transform.scale.x, transform.scale.y, transform.scale.z);
  object.updateMatrixWorld(true);
}

function applyBodyTransform(world: StarterGameplayHostOptions["runtimePhysics"]["world"], body: KinematicPhysicsBody, transform: Transform) {
  rigidBody.setTransform(
    world,
    body,
    [transform.position.x, transform.position.y, transform.position.z],
    new Quaternion().setFromEuler(new Euler(transform.rotation.x, transform.rotation.y, transform.rotation.z))
      .toArray()
      .slice(0, 4) as [number, number, number, number],
    true
  );
}
