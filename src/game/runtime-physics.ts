import { deriveRenderScene, type DerivedRenderMesh, type DerivedRenderScene } from "@ggez/render-pipeline";
import type { Material } from "@ggez/shared";
import type { ThreeRuntimeSceneInstance } from "@ggez/three-runtime";
import {
  box,
  cylinder,
  dof,
  massProperties,
  MotionQuality,
  MotionType,
  rigidBody,
  sphere,
  triangleMesh,
  type Shape
} from "crashcat";
import { BufferGeometry, ConeGeometry, CylinderGeometry, Float32BufferAttribute, Matrix4, Quaternion, SphereGeometry, Vector3 } from "three";
import { BoxGeometry } from "three";
import { PhysicsBody, PhysicsWorld, setQuaternionFromPhysics, setVector3FromPhysics, toPhysicsQuatFromEuler, toPhysicsVec3 } from "./physics";

export type RuntimePhysicsSession = {
  colliderCount: number;
  dispose: () => void;
  getBody: (nodeId: string) => PhysicsBody | undefined;
  renderScene: DerivedRenderScene;
  syncVisuals: () => void;
  world: PhysicsWorld;
};

export function createRuntimePhysicsSession(options: {
  runtimeScene: ThreeRuntimeSceneInstance;
  world: PhysicsWorld;
}): RuntimePhysicsSession {
  const renderScene = deriveRuntimeRenderScene(options.runtimeScene);
  const physicsMeshes = renderScene.meshes.filter((mesh) => mesh.physics?.enabled);
  const physicsMeshIds = new Set(physicsMeshes.map((mesh) => mesh.nodeId));
  const staticMeshes = renderScene.meshes.filter((mesh) => !physicsMeshIds.has(mesh.nodeId));
  const bodiesByNodeId = new Map<string, PhysicsBody>();
  const managedBodies: PhysicsBody[] = [];
  const dynamicBindings: Array<{
    body: PhysicsBody;
    object: NonNullable<ReturnType<ThreeRuntimeSceneInstance["nodesById"]["get"]>>;
  }> = [];

  staticMeshes.forEach((mesh) => {
    const body = createStaticRigidBody(options.world, mesh);
    managedBodies.push(body);
    bodiesByNodeId.set(mesh.nodeId, body);
  });

  physicsMeshes.forEach((mesh) => {
    const body = createConfiguredRigidBody(options.world, mesh);
    managedBodies.push(body);
    bodiesByNodeId.set(mesh.nodeId, body);

    if (body.motionType === MotionType.DYNAMIC) {
      const object = options.runtimeScene.nodesById.get(mesh.nodeId);

      if (object) {
        dynamicBindings.push({ body, object });
      }
    }
  });

  return {
    colliderCount: staticMeshes.length + physicsMeshes.length,
    dispose() {
      managedBodies.forEach((body) => {
        rigidBody.remove(options.world, body);
      });
      managedBodies.length = 0;
      bodiesByNodeId.clear();
      dynamicBindings.length = 0;
    },
    getBody(nodeId) {
      return bodiesByNodeId.get(nodeId);
    },
    renderScene,
    syncVisuals() {
      dynamicBindings.forEach(({ body, object }) => {
        scratchWorldMatrix.compose(
          setVector3FromPhysics(scratchPosition, body.position),
          setQuaternionFromPhysics(scratchQuaternion, body.quaternion),
          object.scale
        );

        if (object.parent) {
          object.parent.updateMatrixWorld(true);
          scratchLocalMatrix.copy(object.parent.matrixWorld).invert().multiply(scratchWorldMatrix);
          scratchLocalMatrix.decompose(object.position, object.quaternion, scratchScale);
          object.scale.copy(scratchScale);
          return;
        }

        scratchWorldMatrix.decompose(object.position, object.quaternion, scratchScale);
        object.scale.copy(scratchScale);
      });
    },
    world: options.world
  };
}

function createStaticRigidBody(world: PhysicsWorld, mesh: DerivedRenderMesh) {
  return rigidBody.create(world, {
    motionType: MotionType.STATIC,
    objectLayer: world.gameLayers.staticObjectLayer,
    position: toPhysicsVec3(mesh.position),
    quaternion: toPhysicsQuatFromEuler(mesh.rotation),
    shape: createMeshShape(mesh, false).shape
  });
}

function createConfiguredRigidBody(world: PhysicsWorld, mesh: DerivedRenderMesh) {
  const physics = mesh.physics;
  const motionType =
    physics?.bodyType === "fixed"
      ? MotionType.STATIC
      : physics?.bodyType === "kinematicPosition"
        ? MotionType.KINEMATIC
        : MotionType.DYNAMIC;
  const shapeResult = createMeshShape(mesh, true);
  const density = physics?.density ?? 1000;
  const bodySettings: Parameters<typeof rigidBody.create>[1] = {
    allowSleeping: physics?.canSleep ?? true,
    angularDamping: physics?.angularDamping ?? 0,
    collisionMask: 0xffffffff,
    collisionGroups: 0xffffffff,
    collideKinematicVsNonDynamic: motionType === MotionType.KINEMATIC,
    friction: physics?.friction ?? 0.7,
    gravityFactor: physics?.gravityScale ?? 1,
    linearDamping: physics?.linearDamping ?? 0,
    motionQuality: physics?.ccd ? MotionQuality.LINEAR_CAST : MotionQuality.DISCRETE,
    motionType,
    objectLayer:
      motionType === MotionType.STATIC ? world.gameLayers.staticObjectLayer : world.gameLayers.movingObjectLayer,
    position: toPhysicsVec3(mesh.position),
    quaternion: toPhysicsQuatFromEuler(mesh.rotation),
    restitution: physics?.restitution ?? 0,
    sensor: physics?.sensor ?? false,
    shape: shapeResult.shape,
    allowedDegreesOfFreedom: dof(
      !(physics?.lockTranslations ?? false),
      !(physics?.lockTranslations ?? false),
      !(physics?.lockTranslations ?? false),
      !(physics?.lockRotations ?? false),
      !(physics?.lockRotations ?? false),
      !(physics?.lockRotations ?? false)
    )
  };

  if (motionType === MotionType.DYNAMIC) {
    if (shapeResult.boundsSize && shapeResult.usedTriangleMesh) {
      const override = massProperties.create();
      massProperties.setMassAndInertiaOfSolidBox(override, shapeResult.boundsSize, density);

      if (typeof physics?.mass === "number") {
        massProperties.scaleToMass(override, physics.mass);
      }

      bodySettings.massPropertiesOverride = override;
    } else if (typeof physics?.mass === "number") {
      bodySettings.mass = physics.mass;
    }
  }

  return rigidBody.create(world, bodySettings);
}

function createMeshShape(mesh: DerivedRenderMesh, preferHints: boolean): {
  boundsSize?: [number, number, number];
  shape: Shape;
  usedTriangleMesh: boolean;
} {
  const physics = mesh.physics;
  const geometry = createRenderableGeometry(mesh);

  if (!geometry) {
    return {
      shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
      usedTriangleMesh: false
    };
  }

  const prepared = prepareGeometry(mesh, geometry);
  geometry.dispose();

  if (preferHints && physics?.colliderShape === "ball") {
    const radius = Math.max(prepared.boundsSize[0], prepared.boundsSize[1], prepared.boundsSize[2]) * 0.5;
    return {
      boundsSize: prepared.boundsSize,
      shape: sphere.create({ density: physics.density, radius }),
      usedTriangleMesh: false
    };
  }

  if (preferHints && physics?.colliderShape === "cuboid") {
    return {
      boundsSize: prepared.boundsSize,
      shape: box.create({
        convexRadius: 0.02,
        density: physics.density,
        halfExtents: [
          prepared.boundsSize[0] * 0.5,
          prepared.boundsSize[1] * 0.5,
          prepared.boundsSize[2] * 0.5
        ]
      }),
      usedTriangleMesh: false
    };
  }

  if (preferHints && physics?.colliderShape === "cylinder") {
    return {
      boundsSize: prepared.boundsSize,
      shape: cylinder.create({
        density: physics.density,
        halfHeight: prepared.boundsSize[1] * 0.5,
        radius: Math.max(prepared.boundsSize[0], prepared.boundsSize[2]) * 0.5
      }),
      usedTriangleMesh: false
    };
  }

  return {
    boundsSize: prepared.boundsSize,
    shape: triangleMesh.create({
      indices: prepared.indices,
      positions: prepared.positions
    }),
    usedTriangleMesh: true
  };
}

function prepareGeometry(mesh: DerivedRenderMesh, geometry: BufferGeometry) {
  geometry.computeBoundingBox();
  const position = geometry.getAttribute("position");
  const index = geometry.getIndex();
  const pivot = mesh.pivot ?? { x: 0, y: 0, z: 0 };
  const positions = new Array<number>(position.count * 3);
  const boundsMin = [Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY];
  const boundsMax = [Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY];

  for (let vertexIndex = 0; vertexIndex < position.count; vertexIndex += 1) {
    const x = position.getX(vertexIndex) * mesh.scale.x - pivot.x;
    const y = position.getY(vertexIndex) * mesh.scale.y - pivot.y;
    const z = position.getZ(vertexIndex) * mesh.scale.z - pivot.z;
    positions[vertexIndex * 3] = x;
    positions[vertexIndex * 3 + 1] = y;
    positions[vertexIndex * 3 + 2] = z;
    boundsMin[0] = Math.min(boundsMin[0], x);
    boundsMin[1] = Math.min(boundsMin[1], y);
    boundsMin[2] = Math.min(boundsMin[2], z);
    boundsMax[0] = Math.max(boundsMax[0], x);
    boundsMax[1] = Math.max(boundsMax[1], y);
    boundsMax[2] = Math.max(boundsMax[2], z);
  }

  return {
    boundsSize: [
      Math.max(0.01, boundsMax[0] - boundsMin[0]),
      Math.max(0.01, boundsMax[1] - boundsMin[1]),
      Math.max(0.01, boundsMax[2] - boundsMin[2])
    ] as [number, number, number],
    indices: index ? Array.from(index.array as ArrayLike<number>) : Array.from({ length: position.count }, (_, value) => value),
    positions
  };
}

function deriveRuntimeRenderScene(runtimeScene: ThreeRuntimeSceneInstance): DerivedRenderScene {
  return deriveRenderScene(
    runtimeScene.scene.nodes,
    runtimeScene.scene.entities,
    runtimeScene.scene.materials.map(toSharedMaterial),
    runtimeScene.scene.assets
  );
}

function toSharedMaterial(material: ThreeRuntimeSceneInstance["scene"]["materials"][number]): Material {
  return {
    color: material.color,
    colorTexture: material.baseColorTexture,
    id: material.id,
    metalness: material.metallicFactor,
    metalnessTexture: material.metallicRoughnessTexture,
    name: material.name,
    normalTexture: material.normalTexture,
    roughness: material.roughnessFactor,
    roughnessTexture: material.metallicRoughnessTexture,
    side: material.side
  };
}

function createRenderableGeometry(mesh: DerivedRenderMesh) {
  let geometry: BufferGeometry | undefined;

  if (mesh.surface) {
    geometry = createIndexedGeometry(mesh.surface.positions, mesh.surface.indices, mesh.surface.uvs, mesh.surface.groups);
  } else if (mesh.primitive?.kind === "box") {
    geometry = new BoxGeometry(mesh.primitive.size.x, mesh.primitive.size.y, mesh.primitive.size.z);
  } else if (mesh.primitive?.kind === "sphere") {
    geometry = new SphereGeometry(mesh.primitive.radius, mesh.primitive.widthSegments, mesh.primitive.heightSegments);
  } else if (mesh.primitive?.kind === "cylinder") {
    geometry = new CylinderGeometry(
      mesh.primitive.radiusTop,
      mesh.primitive.radiusBottom,
      mesh.primitive.height,
      mesh.primitive.radialSegments
    );
  } else if (mesh.primitive?.kind === "cone") {
    geometry = new ConeGeometry(mesh.primitive.radius, mesh.primitive.height, mesh.primitive.radialSegments);
  }

  return geometry;
}

function createIndexedGeometry(
  positions: number[],
  indices: number[] | undefined,
  uvs: number[] | undefined,
  groups: Array<{ count: number; materialIndex: number; start: number }> | undefined
) {
  const geometry = new BufferGeometry();
  geometry.setAttribute("position", new Float32BufferAttribute(positions, 3));

  if (uvs) {
    geometry.setAttribute("uv", new Float32BufferAttribute(uvs, 2));
  }

  if (indices) {
    geometry.setIndex(indices);
  }

  geometry.clearGroups();
  groups?.forEach((group) => {
    geometry.addGroup(group.start, group.count, group.materialIndex);
  });
  return geometry;
}

const scratchLocalMatrix = new Matrix4();
const scratchPosition = new Vector3();
const scratchQuaternion = new Quaternion();
const scratchScale = new Vector3();
const scratchWorldMatrix = new Matrix4();
