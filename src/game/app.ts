import {
  createGameplayRuntime,
  createGameplayRuntimeSceneFromRuntimeScene,
  type GameplayRuntime,
  type GameplayRuntimeSystemRegistration
} from "@ggez/gameplay-runtime";
import { createRapierPhysicsWorld, ensureRapierRuntimePhysics } from "@ggez/runtime-physics-rapier";
import { createThreeRuntimeSceneInstance, type ThreeRuntimeSceneInstance } from "@ggez/three-runtime";
import * as THREE from "three";
import { frameCameraOnObject } from "./camera";
import { createDefaultGameplaySystems, createStarterGameplayHost, mergeGameplaySystems } from "./gameplay";
import { createRuntimePhysicsSession, type RuntimePhysicsSession } from "./runtime-physics";
import type { GameSceneBootstrapContext, GameSceneDefinition, GameSceneLifecycle } from "./scene-types";
import type RAPIER from "@dimforge/rapier3d-compat";
import type { StarterPlayerController } from "./starter-player-controller";

type GameAppOptions = {
  initialSceneId: string;
  root: HTMLDivElement;
  scenes: Record<string, GameSceneDefinition>;
};

type ActiveScene = {
  accumulatorSeconds: number;
  gameplayRuntime: GameplayRuntime;
  id: string;
  lifecycle: GameSceneLifecycle;
  player: StarterPlayerController | null;
  physicsWorld: RAPIER.World;
  runtimePhysics: RuntimePhysicsSession;
  runtimeScene: ThreeRuntimeSceneInstance;
};

const FIXED_STEP_SECONDS = 1 / 60;
const MAX_PHYSICS_CATCH_UP_SECONDS = FIXED_STEP_SECONDS * 4;

export function createGameApp(options: GameAppOptions) {
  options.root.innerHTML = `
    <div class="game-shell">
      <div class="game-loading is-visible" data-game-loading>
        <div class="game-loading-inner">
          <div class="game-loading-title">BIKE LIFE</div>
          <div class="game-loading-subtitle">tong hawk style</div>
          <div class="game-loading-bar">
            <div class="game-loading-bar-fill" data-game-loading-fill></div>
          </div>
          <div class="game-loading-label" data-game-loading-label>loading</div>
        </div>
      </div>
      <div class="game-status" data-game-status hidden></div>
    </div>
  `;

  const shell = options.root.querySelector<HTMLDivElement>(".game-shell");
  const loading = options.root.querySelector<HTMLDivElement>("[data-game-loading]");
  const loadingFill = options.root.querySelector<HTMLDivElement>("[data-game-loading-fill]");
  const loadingLabel = options.root.querySelector<HTMLDivElement>("[data-game-loading-label]");
  const status = options.root.querySelector<HTMLDivElement>("[data-game-status]");

  if (!shell || !loading || !loadingFill || !loadingLabel || !status) {
    throw new Error("Failed to initialize game shell.");
  }

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.max(0.5, Math.min(window.devicePixelRatio, 2) * 0.5));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  shell.append(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 4000);
  const clock = new THREE.Clock();
  let activeScene: ActiveScene | undefined;
  let currentLoadToken = 0;
  let disposed = false;

  const setStatus = (message: string) => {
    status.hidden = message.length === 0;
    status.textContent = message;
  };

  const setLoadingState = (input: {
    label?: string;
    progress?: number;
    visible?: boolean;
  }) => {
    if (typeof input.progress === "number") {
      loadingFill.style.transform = `scaleX(${THREE.MathUtils.clamp(input.progress, 0, 1)})`;
    }

    if (typeof input.label === "string") {
      loadingLabel.textContent = input.label;
    }

    if (typeof input.visible === "boolean") {
      loading.classList.toggle("is-visible", input.visible);
      loading.classList.toggle("is-hidden", !input.visible);
    }
  };

  const stepActiveScene = (deltaSeconds: number) => {
    if (!activeScene) {
      return;
    }

    activeScene.accumulatorSeconds = Math.min(
      activeScene.accumulatorSeconds + deltaSeconds,
      MAX_PHYSICS_CATCH_UP_SECONDS
    );

    while (activeScene.accumulatorSeconds >= FIXED_STEP_SECONDS) {
      activeScene.player?.updateBeforeStep(FIXED_STEP_SECONDS);
      activeScene.lifecycle.fixedUpdate?.(FIXED_STEP_SECONDS);
      activeScene.physicsWorld.step();
      activeScene.runtimePhysics.syncVisuals();
      activeScene.player?.updateAfterStep(FIXED_STEP_SECONDS);
      activeScene.accumulatorSeconds -= FIXED_STEP_SECONDS;
    }
  };

  const renderFrame = () => {
    if (disposed) {
      return;
    }

    requestAnimationFrame(renderFrame);
    const delta = Math.min(clock.getDelta(), 0.1);
    stepActiveScene(delta);
    activeScene?.lifecycle.update?.(delta);
    activeScene?.gameplayRuntime.update(delta);
    renderer.render(scene, camera);
  };

  const disposeActiveScene = async () => {
    if (!activeScene) {
      return;
    }

    const sceneToDispose = activeScene;
    activeScene = undefined;
    scene.remove(sceneToDispose.runtimeScene.root);
    if (sceneToDispose.player) {
      scene.remove(sceneToDispose.player.object);
    }
    await sceneToDispose.lifecycle.dispose?.();
    sceneToDispose.player?.dispose();
    sceneToDispose.gameplayRuntime.dispose();
    sceneToDispose.runtimeScene.dispose();
    sceneToDispose.runtimePhysics.dispose();
    sceneToDispose.physicsWorld.free();
  };

  const loadScene = async (sceneId: string) => {
    const definition = options.scenes[sceneId];

    if (!definition) {
      throw new Error(`Unknown scene "${sceneId}".`);
    }

    const loadToken = ++currentLoadToken;
    setStatus("");
    setLoadingState({
      label: "loading",
      progress: 0.04,
      visible: true
    });

    try {
      await ensureRapierRuntimePhysics();
      setLoadingState({ progress: 0.16 });
      const runtimeManifest = await definition.source.load();

      if (disposed || loadToken !== currentLoadToken) {
        return;
      }

      setLoadingState({ progress: 0.34 });
      const runtimeScene = await createThreeRuntimeSceneInstance(runtimeManifest, {
        applyToScene: scene,
        resolveAssetUrl: ({ path }) => path
      });
      setLoadingState({ progress: 0.56 });
      const physicsWorld = createRapierPhysicsWorld(runtimeScene.scene.settings);
      const runtimePhysics = createRuntimePhysicsSession({
        runtimeScene,
        world: physicsWorld
      });
      const gameplayHost = createStarterGameplayHost({
        runtimePhysics,
        runtimeScene
      });
      const bootstrapContext = createBootstrapContext({
        camera,
        gotoScene: loadScene,
        physicsWorld,
        renderer,
        runtimeScene,
        scene,
        sceneId,
        setStatus
      });
      const systems = resolveSceneSystems(definition, bootstrapContext);
      const gameplayRuntime = createGameplayRuntime({
        host: gameplayHost,
        scene: createGameplayRuntimeSceneFromRuntimeScene(runtimeScene.scene),
        systems
      });
      setLoadingState({ progress: 0.72 });
      const player = await createStarterPlayerController({
        camera,
        definition,
        domElement: renderer.domElement,
        gameplayRuntime,
        physicsWorld,
        runtimeScene,
        setStatus
      });
      setLoadingState({ progress: 0.84 });

      gameplayRuntime.start();
      scene.add(runtimeScene.root);
      if (player) {
        scene.add(player.object);
        player.updateAfterStep(FIXED_STEP_SECONDS);
      } else {
        frameCameraOnObject(camera, runtimeScene.root);
      }

      let customStatusApplied = false;
      const sceneSetStatus = (message: string) => {
        customStatusApplied = true;
        setStatus(message);
      };

      const lifecycle =
        (await definition.mount?.({
          camera,
          gameplayRuntime,
          gotoScene: loadScene,
          player,
          physicsWorld,
          renderer,
          runtimePhysics,
          runtimeScene,
          scene,
          sceneId,
          sceneSettings: runtimeScene.scene.settings,
          setStatus: sceneSetStatus
        })) ?? {};
      setLoadingState({ progress: 0.96 });

      if (disposed || loadToken !== currentLoadToken) {
        scene.remove(runtimeScene.root);
        if (player) {
          scene.remove(player.object);
        }
        await lifecycle.dispose?.();
        player?.dispose();
        gameplayRuntime.dispose();
        runtimeScene.dispose();
        runtimePhysics.dispose();
        physicsWorld.free();
        return;
      }

      const previousScene = activeScene;
      activeScene = {
        accumulatorSeconds: 0,
        gameplayRuntime,
        id: sceneId,
        lifecycle,
        player,
        physicsWorld,
        runtimePhysics,
        runtimeScene
      };

      if (previousScene) {
        scene.remove(previousScene.runtimeScene.root);
        if (previousScene.player) {
          scene.remove(previousScene.player.object);
        }
        await previousScene.lifecycle.dispose?.();
        previousScene.player?.dispose();
        previousScene.gameplayRuntime.dispose();
        previousScene.runtimeScene.dispose();
        previousScene.runtimePhysics.dispose();
        previousScene.physicsWorld.free();
      }

      if (!customStatusApplied) {
        setStatus("");
      }
      setLoadingState({ progress: 1 });
      requestAnimationFrame(() => {
        if (!disposed && loadToken === currentLoadToken) {
          setLoadingState({
            label: "loading",
            visible: false
          });
        }
      });

    } catch (error) {
      setLoadingState({
        label: "loading failed",
        progress: 1,
        visible: true
      });
      setStatus(`Failed to load ${definition.title}.`);
      throw error;
    }
  };

  const start = () => loadScene(options.initialSceneId);

  const dispose = async () => {
    disposed = true;
    window.removeEventListener("resize", handleResize);
    await disposeActiveScene();
    renderer.dispose();
  };

  const handleResize = () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  };

  window.addEventListener("resize", handleResize);
  renderFrame();

  return {
    camera,
    dispose,
    initialSceneId: options.initialSceneId,
    loadScene,
    renderer,
    scene,
    start,
    setStatus
  };
}

function createBootstrapContext(options: {
  camera: THREE.PerspectiveCamera;
  gotoScene: (sceneId: string) => Promise<void>;
  physicsWorld: RAPIER.World;
  renderer: THREE.WebGLRenderer;
  runtimeScene: ThreeRuntimeSceneInstance;
  scene: THREE.Scene;
  sceneId: string;
  setStatus: (message: string) => void;
}): GameSceneBootstrapContext {
  return {
    camera: options.camera,
    gotoScene: options.gotoScene,
    physicsWorld: options.physicsWorld,
    renderer: options.renderer,
    runtimeScene: options.runtimeScene,
    scene: options.scene,
    sceneId: options.sceneId,
    sceneSettings: options.runtimeScene.scene.settings,
    setStatus: options.setStatus
  };
}

async function createStarterPlayerController(options: {
  camera: THREE.PerspectiveCamera;
  definition: GameSceneDefinition;
  domElement: HTMLCanvasElement;
  gameplayRuntime: GameplayRuntime;
  physicsWorld: RAPIER.World;
  runtimeScene: ThreeRuntimeSceneInstance;
  setStatus: (message: string) => void;
}) {
  const { StarterPlayerController } = await import("./starter-player-controller");

  if (options.definition.player === false) {
    return null;
  }

  const playerConfig = options.definition.player ?? {};
  const playerSpawn = options.runtimeScene.entities.find((entity) => {
    if (entity.type !== "player-spawn") {
      return false;
    }

    return playerConfig.spawnEntityId ? entity.id === playerConfig.spawnEntityId : true;
  });

  if (!playerSpawn) {
    return null;
  }

  return StarterPlayerController.create({
    camera: options.camera,
    cameraMode: playerConfig.cameraMode ?? options.runtimeScene.scene.settings.player.cameraMode,
    domElement: options.domElement,
    gameplayRuntime: options.gameplayRuntime,
    sceneSettings: options.runtimeScene.scene.settings,
    setStatus: options.setStatus,
    spawn: {
      position: playerSpawn.transform.position,
      rotationY: playerSpawn.transform.rotation.y
    },
    world: options.physicsWorld
  });
}

function resolveSceneSystems(definition: GameSceneDefinition, context: GameSceneBootstrapContext): GameplayRuntimeSystemRegistration[] {
  const starterSystems = createDefaultGameplaySystems(context.sceneSettings);

  if (!definition.systems) {
    return starterSystems;
  }

  const sceneSystems = typeof definition.systems === "function" ? definition.systems(context) : definition.systems;
  return mergeGameplaySystems(starterSystems, sceneSystems);
}
