import type { GameplayRuntime } from "@ggez/gameplay-runtime";
import { copyPose, createPoseBufferFromRig, type PoseBuffer } from "@ggez/anim-core";
import { createAnimatorInstance } from "@ggez/anim-runtime";
import { vec3, type SceneSettings, type Vec3 } from "@ggez/shared";
import { capsule, dof, MotionQuality, MotionType, rigidBody } from "crashcat";
import nipplejs from "nipplejs";
import { Bone, Box3, Euler, Group, MathUtils, PerspectiveCamera, Quaternion, Vector3, type Object3D, type Skeleton } from "three";
import { animations } from "../animations";
import bikeSoundUrl from "../assets/bikesound.mp3?url";
import { BikePhysicsRig } from "./bike-physics";
import { raycastClosest, setVector3FromPhysics, type PhysicsBody, type PhysicsWorld } from "./physics";

type StarterPlayerSpawn = {
  position: Vec3;
  rotationY: number;
};

export type StarterPlayerBike = {
  frontWheelAnchor?: { x: number; y: number; z: number };
  groundOffset?: number;
  mountRadius?: number;
  object: Object3D;
  rearWheelAnchor?: { x: number; y: number; z: number };
  wheelRadius?: number;
  wheels?: Object3D[];
};

type RegisteredBike = BikePhysicsRig;

type StarterPlayerControllerOptions = {
  camera: PerspectiveCamera;
  cameraMode: SceneSettings["player"]["cameraMode"];
  domElement: HTMLCanvasElement;
  gameplayRuntime: GameplayRuntime;
  sceneSettings: Pick<SceneSettings, "player" | "world">;
  setStatus: (message: string) => void;
  spawn: StarterPlayerSpawn;
  world: PhysicsWorld;
};

const BIKE_CRUISE_SPEED_BONUS = 2;
const BIKE_INTERACT_KEY = "KeyF";
const BIKE_PITCH_RESPONSE = 7;
const BIKE_REVERSE_MULTIPLIER = 0.45;
const BIKE_ROLL_RESPONSE = 8;
const BIKE_WHEELIE_RESPONSE = 6;
const DEFAULT_BIKE_MOUNT_RADIUS = 2.4;
const FOOT_AIR_ACCELERATION = 2.8;
const FOOT_AIR_DECELERATION = 1.8;
const FOOT_ACCELERATION = 15;
const FOOT_DECELERATION = 18;
const GROUND_MIN_NORMAL_Y = 0.45;
const GROUND_PROBE_DISTANCE = 0.2;
const GROUND_PROBE_HEIGHT = 0.12;
const JUMP_GROUND_LOCK_SECONDS = 0.12;
const MOUNTED_CAMERA_EYE_RESPONSE = 4.6;
const MOUNTED_CAMERA_FOCUS_RESPONSE = 5.2;
const ON_FOOT_ANIMATION_PLAYBACK_SCALE = 2;
const PLAYER_SCALE_FACTOR = 0.5;
const RUN_SPEED_MULTIPLIER = 1.2;
const UNMOUNTED_CAMERA_EYE_RESPONSE = 9;
const UNMOUNTED_CAMERA_FOCUS_RESPONSE = 10;
const WALK_SPEED_MULTIPLIER = 0.72;

export class StarterPlayerController {
  readonly object = new Group();

  private readonly body: PhysicsBody;
  private readonly camera: PerspectiveCamera;
  private cameraMode: SceneSettings["player"]["cameraMode"];
  private readonly domElement: HTMLCanvasElement;
  private readonly footOffset: number;
  private readonly gameplayRuntime: GameplayRuntime;
  private readonly halfHeight: number;
  private readonly isTouchDevice: boolean;
  private interactQueued = false;
  private jumpGroundLockRemaining = 0;
  private jumpQueued = false;
  private readonly keyState = new Set<string>();
  private mobileBikeButton: HTMLButtonElement | null = null;
  private mobileControlsRoot: HTMLDivElement | null = null;
  private mobileJoystick: ReturnType<typeof nipplejs.create> | null = null;
  private mobileJoystickZone: HTMLDivElement | null = null;
  private mobileMoveX = 0;
  private mobileMoveY = 0;
  private mobileOrbitPointerId: number | null = null;
  private mobileOrbitLastX = 0;
  private mobileOrbitLastY = 0;
  private mobileWheelieButton: HTMLButtonElement | null = null;
  private mobileWheelieHeld = false;
  private orbitYaw = 0;
  private pitch = 0;
  private pointerLocked = false;
  private readonly radius: number;
  private readonly sceneSettings: Pick<SceneSettings, "player" | "world">;
  private readonly setStatus: (message: string) => void;
  private readonly smoothedCameraEye = new Vector3();
  private readonly smoothedCameraFocus = new Vector3();
  private readonly standingHeight: number;
  private readonly supportVelocity = new Vector3();
  private readonly targetPlanarVelocity = new Vector3();
  private readonly planarVelocity = new Vector3();
  private readonly visualRoot = new Group();
  private readonly world: PhysicsWorld;
  private bikeAudioBufferPromise: Promise<AudioBuffer> | null = null;
  private bikeAudioContext: AudioContext | null = null;
  private bikeAudioGainNode: GainNode | null = null;
  private bikeAudioSource: AudioBufferSourceNode | null = null;
  private animationAnimator: ReturnType<typeof createAnimatorInstance> | null = null;
  private animationBaseOffset = new Vector3();
  private animationBonesByName = new Map<string, Bone>();
  private animationDisplayPose: PoseBuffer | null = null;
  private animationModeValue = 0;
  private animationObject: Object3D | null = null;
  private animationRootCompensation = new Vector3();
  private animationScale = 1;
  private animationSkeletons: Skeleton[] = [];
  private animationSpeedValue = 0;
  private bike: RegisteredBike | null = null;
  private mountedHandleBones: Bone[] = [];
  private mountedBikeLean = 0;
  private mountedBikePitch = 0;
  private mountedSurfacePitch = 0;
  private mountedSurfaceRoll = 0;
  private mountedBikeWheelie = 0;
  private mountedCounterTiltBones: Bone[] = [];
  private mountedThrottleInput = 0;
  private mountedTurnInput = 0;
  private mountedBike: RegisteredBike | null = null;
  private yaw = 0;

  static async create(options: StarterPlayerControllerOptions) {
    const controller = new StarterPlayerController(options);
    await controller.initializeAnimatedCharacter();
    return controller;
  }

  constructor(options: StarterPlayerControllerOptions) {
    this.camera = options.camera;
    this.cameraMode = options.cameraMode;
    this.domElement = options.domElement;
    this.gameplayRuntime = options.gameplayRuntime;
    this.sceneSettings = options.sceneSettings;
    this.setStatus = options.setStatus;
    this.world = options.world;
    this.isTouchDevice = isLikelyTouchDevice();
    this.standingHeight = Math.max(0.9, options.sceneSettings.player.height * PLAYER_SCALE_FACTOR);
    this.radius = MathUtils.clamp(this.standingHeight * 0.18, 0.24, 0.42);
    this.halfHeight = Math.max(0.12, this.standingHeight * 0.5 - this.radius);
    this.footOffset = this.halfHeight + this.radius;
    this.yaw = options.spawn.rotationY;
    this.orbitYaw = options.spawn.rotationY;
    this.pitch = defaultPitchForCameraMode(this.cameraMode);

    const spawnPosition = {
      x: options.spawn.position.x,
      y: options.spawn.position.y + this.standingHeight * 0.5 + 0.04,
      z: options.spawn.position.z
    };
    this.body = rigidBody.create(this.world, {
      allowSleeping: false,
      allowedDegreesOfFreedom: dof(true, true, true, false, false, false),
      friction: 0,
      linearDamping: 0.8,
      motionQuality: MotionQuality.LINEAR_CAST,
      motionType: MotionType.DYNAMIC,
      objectLayer: this.world.gameLayers.movingObjectLayer,
      position: [spawnPosition.x, spawnPosition.y, spawnPosition.z],
      shape: capsule.create({ halfHeightOfCylinder: this.halfHeight, radius: this.radius })
    });

    this.object.add(this.visualRoot);
    this.object.position.set(spawnPosition.x, spawnPosition.y, spawnPosition.z);
    this.visualRoot.rotation.y = this.yaw;
    this.smoothedCameraEye.copy(this.object.position).y += this.standingHeight * 0.42;
    this.smoothedCameraFocus.copy(this.smoothedCameraEye);

    this.domElement.addEventListener("click", this.handleCanvasClick);
    window.addEventListener("blur", this.handleWindowBlur);
    window.addEventListener("keydown", this.handleKeyDown);
    window.addEventListener("keyup", this.handleKeyUp);
    window.addEventListener("mousemove", this.handleMouseMove);

    if (this.isTouchDevice) {
      this.initializeMobileControls();
    }
  }

  attachBike(bike: StarterPlayerBike | null) {
    this.bike?.dispose();
    this.bike = bike
      ? new BikePhysicsRig({
          frontWheelAnchor: new Vector3(
            bike.frontWheelAnchor?.x ?? 0,
            bike.frontWheelAnchor?.y ?? 0,
            bike.frontWheelAnchor?.z ?? 0
          ),
          groundOffset: bike.groundOffset ?? 0,
          mountRadius: bike.mountRadius ?? DEFAULT_BIKE_MOUNT_RADIUS,
          object: bike.object,
          rearWheelAnchor: new Vector3(
            bike.rearWheelAnchor?.x ?? 0,
            bike.rearWheelAnchor?.y ?? 0,
            bike.rearWheelAnchor?.z ?? 0
          ),
          wheelRadius: bike.wheelRadius ?? 0.34,
          wheels: bike.wheels ?? [],
          world: this.world
        })
      : null;

    if (!this.bike) {
      if (this.mountedBike) {
        this.setPlayerMountedState(false);
      }
      this.mountedBike = null;
      this.stopBikeAudio();
    }
  }

  dispose() {
    this.releasePointerLock();
    this.domElement.removeEventListener("click", this.handleCanvasClick);
    window.removeEventListener("blur", this.handleWindowBlur);
    window.removeEventListener("keydown", this.handleKeyDown);
    window.removeEventListener("keyup", this.handleKeyUp);
    window.removeEventListener("mousemove", this.handleMouseMove);
    this.disposeMobileControls();
    this.bike?.dispose();
    this.bike = null;
    this.mountedBike = null;
    this.disposeBikeAudio();
    this.gameplayRuntime.removeActor("player");
    this.setStatus("");
  }

  releasePointerLock() {
    if (document.pointerLockElement === this.domElement) {
      document.exitPointerLock();
    }

    this.pointerLocked = false;
  }

  setCameraMode(cameraMode: SceneSettings["player"]["cameraMode"]) {
    this.cameraMode = cameraMode;
  }

  updateAfterStep(deltaSeconds: number) {
    const translation = setVector3FromPhysics(scratchBodyTranslation, this.body.position);
    this.bike?.syncVisuals(deltaSeconds);

    if (this.mountedBike) {
      this.mountedBike.getRiderPosition(scratchMountedRiderPosition);
      this.mountedBike.getRiderQuaternion(scratchMountedBikeQuaternion);
      this.mountedBike.getOrientationEuler(scratchMountedBikeOrientation);
      rigidBody.setTransform(
        this.world,
        this.body,
        [scratchMountedRiderPosition.x, scratchMountedRiderPosition.y, scratchMountedRiderPosition.z],
        [0, 0, 0, 1],
        true
      );
      translation.copy(scratchMountedRiderPosition);
      this.mountedSurfacePitch = scratchMountedBikeOrientation.x;
      this.yaw = scratchMountedBikeOrientation.y;
      this.mountedSurfaceRoll = scratchMountedBikeOrientation.z;
      this.object.quaternion.copy(scratchMountedBikeQuaternion);
    } else {
      this.object.quaternion.identity();
    }

    this.object.position.copy(translation);
    if (this.isTouchDevice) {
      this.orbitYaw = dampAngle(this.orbitYaw, this.yaw, 12, deltaSeconds);
    }
    if (this.mountedBike) {
      this.visualRoot.rotation.set(0, Math.PI, 0, "YXZ");
    } else {
      this.visualRoot.rotation.set(0, this.yaw, 0);
    }
    this.visualRoot.visible = this.cameraMode !== "fps";

    this.updateBikeAudio();

    const eyePosition = new Vector3(
      translation.x,
      translation.y + this.standingHeight * (this.mountedBike ? 0.5 : 0.42),
      translation.z
    );
    const viewDirection = resolveViewDirection(this.orbitYaw, this.pitch, scratchViewDirection);
    const focusTarget = scratchFocusTarget.copy(eyePosition);

    if (this.mountedBike) {
      focusTarget.addScaledVector(scratchYawForward.set(-Math.sin(this.yaw), 0, -Math.cos(this.yaw)), 0.9);
    }

    const eyeResponse = this.mountedBike ? MOUNTED_CAMERA_EYE_RESPONSE : UNMOUNTED_CAMERA_EYE_RESPONSE;
    const focusResponse = this.mountedBike ? MOUNTED_CAMERA_FOCUS_RESPONSE : UNMOUNTED_CAMERA_FOCUS_RESPONSE;
    this.smoothedCameraEye.lerp(eyePosition, 1 - Math.exp(-deltaSeconds * eyeResponse));
    this.smoothedCameraFocus.lerp(focusTarget, 1 - Math.exp(-deltaSeconds * focusResponse));

    if (this.cameraMode === "fps") {
      const cameraEye = this.mountedBike ? eyePosition : this.smoothedCameraEye;
      const cameraFocus = this.mountedBike ? focusTarget : this.smoothedCameraFocus;
      this.camera.position.copy(cameraEye);
      this.camera.lookAt(scratchCameraLookTarget.copy(cameraFocus).add(viewDirection));
    } else if (this.cameraMode === "third-person") {
      if (this.mountedBike) {
        const targetCameraPosition = scratchTargetCameraPosition.copy(focusTarget).addScaledVector(viewDirection, -1.15);
        targetCameraPosition.y += this.standingHeight * 0.04;
        this.camera.position.lerp(targetCameraPosition, 1 - Math.exp(-deltaSeconds * 8.5));
        this.camera.lookAt(focusTarget);
      } else {
        const followDistance = Math.max(2.15, this.standingHeight * 1.85);
        const targetCameraPosition = scratchTargetCameraPosition.copy(this.smoothedCameraEye).addScaledVector(viewDirection, -followDistance);
        targetCameraPosition.y += this.standingHeight * 0.13;
        this.camera.position.lerp(targetCameraPosition, 1 - Math.exp(-deltaSeconds * 8));
        this.camera.lookAt(this.smoothedCameraFocus);
      }
    } else {
      if (this.mountedBike) {
        const targetCameraPosition = scratchTargetCameraPosition.copy(focusTarget).addScaledVector(viewDirection, -2.9);
        targetCameraPosition.y += this.standingHeight * 0.48;
        this.camera.position.lerp(targetCameraPosition, 1 - Math.exp(-deltaSeconds * 7.25));
        this.camera.lookAt(focusTarget);
      } else {
        const followDistance = Math.max(5.8, this.standingHeight * 3.9);
        const targetCameraPosition = scratchTargetCameraPosition.copy(this.smoothedCameraEye).addScaledVector(viewDirection, -followDistance);
        targetCameraPosition.y += this.standingHeight * 1.18;
        this.camera.position.lerp(targetCameraPosition, 1 - Math.exp(-deltaSeconds * 7));
        this.camera.lookAt(this.smoothedCameraFocus);
      }
    }

    this.updateAnimation(deltaSeconds);
    this.setStatus(this.resolveStatusMessage(translation));

    this.gameplayRuntime.updateActor({
      height: this.standingHeight,
      id: "player",
      position: vec3(translation.x, translation.y, translation.z),
      radius: this.radius,
      tags: ["player"]
    });
  }

  updateBeforeStep(deltaSeconds: number) {
    this.jumpGroundLockRemaining = Math.max(0, this.jumpGroundLockRemaining - deltaSeconds);
    const translation = setVector3FromPhysics(scratchBodyTranslation, this.body.position);

    this.resolveBikeInteraction(translation);

    if (this.mountedBike) {
      this.resolveMountedVelocity(deltaSeconds);
      const bikeControlInput = {
        boost: this.isTouchDevice || this.isRunning(),
        steer: this.mountedTurnInput,
        throttle: this.mountedThrottleInput,
        wheelie: (this.keyState.has("Space") || this.mobileWheelieHeld) && this.mountedThrottleInput > 0
      };
      this.mountedBike.updateBeforeStep(deltaSeconds, bikeControlInput);
      this.mountedBike.getRiderPosition(scratchMountedRiderPosition);
      rigidBody.setTransform(
        this.world,
        this.body,
        [scratchMountedRiderPosition.x, scratchMountedRiderPosition.y, scratchMountedRiderPosition.z],
        [0, 0, 0, 1],
        true
      );
      this.planarVelocity.copy(this.mountedBike.getPlanarVelocity(scratchMountedPlanarVelocity));
      this.targetPlanarVelocity.copy(this.planarVelocity);
      this.supportVelocity.set(0, 0, 0);
      this.jumpQueued = false;
      return;
    }

    this.bike?.updateBeforeStep(deltaSeconds, {
      boost: false,
      steer: 0,
      throttle: 0,
      wheelie: false
    });

    this.resolveOnFootVelocity(deltaSeconds);

    const linearVelocity = setVector3FromPhysics(scratchLinearVelocity, this.body.motionProperties.linearVelocity);
    const groundedHit = this.jumpGroundLockRemaining > 0 ? undefined : this.resolveGroundHit(translation);
    const grounded = groundedHit !== undefined;

    if (groundedHit?.body) {
      this.supportVelocity.set(
        groundedHit.body.motionProperties.linearVelocity[0] ?? 0,
        groundedHit.body.motionProperties.linearVelocity[1] ?? 0,
        groundedHit.body.motionProperties.linearVelocity[2] ?? 0
      );
    } else {
      this.supportVelocity.set(0, 0, 0);
    }

    const planarRate = this.resolvePlanarVelocityRate(grounded, this.targetPlanarVelocity.lengthSq() > 0);
    this.planarVelocity.lerp(this.targetPlanarVelocity, 1 - Math.exp(-deltaSeconds * planarRate));

    if (grounded && this.targetPlanarVelocity.lengthSq() < 1e-5 && this.planarVelocity.lengthSq() < 0.0025) {
      this.planarVelocity.set(0, 0, 0);
    }

    rigidBody.setLinearVelocity(this.world, this.body, [
      this.planarVelocity.x + this.supportVelocity.x,
      grounded && linearVelocity.y <= this.supportVelocity.y ? this.supportVelocity.y : linearVelocity.y,
      this.planarVelocity.z + this.supportVelocity.z
    ]
    );

    if (this.jumpQueued) {
      if (!this.mountedBike && this.sceneSettings.player.canJump && grounded) {
        const gravityMagnitude = Math.max(
          0.001,
          Math.hypot(
            this.sceneSettings.world.gravity.x,
            this.sceneSettings.world.gravity.y,
            this.sceneSettings.world.gravity.z
          )
        );
        const currentVelocity = setVector3FromPhysics(scratchCurrentVelocity, this.body.motionProperties.linearVelocity);
        rigidBody.setLinearVelocity(this.world, this.body, [
          currentVelocity.x,
          this.supportVelocity.y + Math.sqrt(2 * gravityMagnitude * this.sceneSettings.player.jumpHeight),
          currentVelocity.z
        ]);
        this.jumpGroundLockRemaining = JUMP_GROUND_LOCK_SECONDS;
      }

      this.jumpQueued = false;
    }
  }

  private async initializeAnimatedCharacter() {
    const bundleDefinition = animations.player;

    if (!bundleDefinition) {
      throw new Error('Missing "player" animation bundle.');
    }

    const bundle = await bundleDefinition.source.load();
    const character = await bundle.loadCharacterAsset();

    if (!character) {
      throw new Error('Animation bundle "player" is missing a character asset.');
    }

    const clips = await bundle.loadGraphClipAssets(character.skeleton);
    const animator = createAnimatorInstance({
      clips,
      graph: bundle.artifact.graph,
      rig: bundle.rig ?? character.rig
    });

    this.animationAnimator = animator;
    this.animationDisplayPose = createPoseBufferFromRig(animator.rig);
    const animationObject = character.root;
    this.animationObject = animationObject;
    animationObject.rotation.y = Math.PI;
    const skeletonSet = new Set<Skeleton>();
    animationObject.traverse((child: Object3D) => {
      const renderCandidate = child as Object3D & {
        castShadow?: boolean;
        frustumCulled?: boolean;
        receiveShadow?: boolean;
      };
      const skinnedCandidate = child as Object3D & {
        isSkinnedMesh?: boolean;
        skeleton?: Skeleton;
      };

      if (child instanceof Bone) {
        this.animationBonesByName.set(child.name, child);
      }

      if (skinnedCandidate.isSkinnedMesh === true && skinnedCandidate.skeleton) {
        skeletonSet.add(skinnedCandidate.skeleton);
      }

      if ("castShadow" in renderCandidate) {
        renderCandidate.castShadow = true;
      }

      if ("receiveShadow" in renderCandidate) {
        renderCandidate.receiveShadow = true;
      }

      if ("frustumCulled" in renderCandidate) {
        renderCandidate.frustumCulled = false;
      }
    });
    this.animationSkeletons = Array.from(skeletonSet);
    this.mountedCounterTiltBones = Array.from(this.animationBonesByName.values()).filter((bone) =>
      /spine|chest|neck/i.test(bone.name)
    );
    this.mountedHandleBones = Array.from(this.animationBonesByName.values()).filter((bone) =>
      /shoulder|arm|forearm|hand/i.test(bone.name)
    );

    const initialBounds = new Box3().setFromObject(animationObject);
    const initialSize = initialBounds.getSize(new Vector3());
    const rawHeight = Math.max(initialSize.y, 0.001);
    this.animationScale = (this.standingHeight / rawHeight) * 0.98;
    animationObject.scale.setScalar(this.animationScale);
    animationObject.updateMatrixWorld(true);

    const fittedBounds = new Box3().setFromObject(animationObject);
    const fittedCenter = fittedBounds.getCenter(new Vector3());
    this.animationBaseOffset.set(
      -fittedCenter.x,
      -this.footOffset - fittedBounds.min.y,
      -fittedCenter.z
    );
    this.syncAnimationOffset();
    this.visualRoot.add(animationObject);

    this.animationAnimator.setInt("mode", 0);
    this.animationAnimator.setFloat("speed", 0);
    this.updateAnimation(0);
  }

  private axis(primary: string, secondary: string) {
    return this.keyState.has(primary) || this.keyState.has(secondary) ? 1 : 0;
  }

  private initializeMobileControls() {
    const shell = this.domElement.parentElement;

    if (!shell) {
      return;
    }

    const controlsRoot = document.createElement("div");
    controlsRoot.className = "touch-controls";
    controlsRoot.dataset.touchControl = "true";

    const joystickZone = document.createElement("div");
    joystickZone.className = "touch-joystick";
    joystickZone.dataset.touchControl = "true";

    const actionStack = document.createElement("div");
    actionStack.className = "touch-actions";
    actionStack.dataset.touchControl = "true";

    const wheelieButton = document.createElement("button");
    wheelieButton.className = "touch-action touch-action-wheelie";
    wheelieButton.dataset.touchControl = "true";
    wheelieButton.type = "button";
    wheelieButton.textContent = "Wheelie";

    const bikeButton = document.createElement("button");
    bikeButton.className = "touch-action touch-action-bike";
    bikeButton.dataset.touchControl = "true";
    bikeButton.type = "button";
    bikeButton.textContent = "Bike";

    actionStack.append(wheelieButton, bikeButton);
    controlsRoot.append(joystickZone, actionStack);
    shell.append(controlsRoot);

    this.mobileControlsRoot = controlsRoot;
    this.mobileJoystickZone = joystickZone;
    this.mobileWheelieButton = wheelieButton;
    this.mobileBikeButton = bikeButton;
    this.mobileJoystick = nipplejs.create({
      color: "#d8ecff",
      fadeTime: 120,
      maxNumberOfJoysticks: 1,
      mode: "static",
      position: { left: "50%", top: "50%" },
      size: 120,
      zone: joystickZone
    });
    this.mobileJoystick.on("move", (event: { data: { vector: { x: number; y: number } } }) => {
      this.mobileMoveX = event.data.vector.x;
      this.mobileMoveY = event.data.vector.y;
    });
    this.mobileJoystick.on("end hidden", () => {
      this.mobileMoveX = 0;
      this.mobileMoveY = 0;
    });

    bikeButton.addEventListener("pointerdown", this.handleMobileBikeButtonDown);
    wheelieButton.addEventListener("pointerdown", this.handleMobileWheelieDown);
    wheelieButton.addEventListener("pointerup", this.handleMobileWheelieUp);
    wheelieButton.addEventListener("pointercancel", this.handleMobileWheelieUp);
  }

  private disposeMobileControls() {
    const shell = this.domElement.parentElement;
    this.mobileJoystick?.destroy();
    this.mobileJoystick = null;

    if (this.mobileBikeButton) {
      this.mobileBikeButton.removeEventListener("pointerdown", this.handleMobileBikeButtonDown);
    }

    if (this.mobileWheelieButton) {
      this.mobileWheelieButton.removeEventListener("pointerdown", this.handleMobileWheelieDown);
      this.mobileWheelieButton.removeEventListener("pointerup", this.handleMobileWheelieUp);
      this.mobileWheelieButton.removeEventListener("pointercancel", this.handleMobileWheelieUp);
    }

    this.mobileControlsRoot?.remove();
    this.mobileControlsRoot = null;
    this.mobileJoystickZone = null;
    this.mobileBikeButton = null;
    this.mobileWheelieButton = null;
    this.mobileMoveX = 0;
    this.mobileMoveY = 0;
    this.mobileOrbitPointerId = null;
    this.mobileWheelieHeld = false;
  }

  private getMoveRightInput() {
    return MathUtils.clamp(this.mobileMoveX + this.axis("KeyD", "ArrowRight") - this.axis("KeyA", "ArrowLeft"), -1, 1);
  }

  private getMoveForwardInput() {
    return MathUtils.clamp(this.mobileMoveY + this.axis("KeyW", "ArrowUp") - this.axis("KeyS", "ArrowDown"), -1, 1);
  }

  private isRunning() {
    return this.keyState.has("ShiftLeft") || this.keyState.has("ShiftRight");
  }

  private resolveBikeDistance(translation: Vector3) {
    if (!this.bike) {
      return Number.POSITIVE_INFINITY;
    }

    return this.bike.getDistanceTo(translation);
  }

  private resolveBikeInteraction(translation: Vector3) {
    if (!this.interactQueued) {
      return;
    }

    this.interactQueued = false;

    if (this.mountedBike) {
      this.dismountBike(translation);
      return;
    }

    if (!this.bike || this.resolveBikeDistance(translation) > this.bike.mountRadius) {
      return;
    }

    this.mountedBike = this.bike;
    this.mountedBike.setMounted(true);
    this.setPlayerMountedState(true);
    this.planarVelocity.set(0, 0, 0);
    this.targetPlanarVelocity.set(0, 0, 0);
    this.animationRootCompensation.set(0, 0, 0);
    this.animationModeValue = 2;
    this.mountedBikeLean = 0;
    this.mountedBikePitch = 0;
    this.mountedSurfacePitch = 0;
    this.mountedSurfaceRoll = 0;
    this.mountedBikeWheelie = 0;
    this.mountedThrottleInput = 0;
    this.mountedTurnInput = 0;
    this.mountedBike.getRiderPosition(scratchMountedRiderPosition);
    rigidBody.setTransform(
      this.world,
      this.body,
      [scratchMountedRiderPosition.x, scratchMountedRiderPosition.y, scratchMountedRiderPosition.z],
      [0, 0, 0, 1],
      true
    );
    this.yaw = this.mountedBike.getYaw();
    this.orbitYaw = this.yaw;
  }

  private dismountBike(translation: Vector3) {
    if (!this.mountedBike) {
      return;
    }

    const bikeYaw = this.mountedBike.getYaw();
    const right = scratchRight.set(Math.cos(this.yaw), 0, -Math.sin(this.yaw));
    const bikeX = translation.x + right.x * 1.35;
    const bikeZ = translation.z + right.z * 1.35;
    const groundY = this.resolveSurfaceContact(bikeX, bikeZ, translation.y + 2).y + this.standingHeight * 0.5 + 0.04;
    this.mountedBike.setMounted(false);
    this.mountedBike.resetToUpright();
    this.setPlayerMountedState(false);
    rigidBody.setTransform(this.world, this.body, [bikeX, groundY, bikeZ], [0, 0, 0, 1], true);
    rigidBody.setLinearVelocity(this.world, this.body, [0, 0, 0]);
    this.mountedBike = null;
    this.mountedBikeLean = 0;
    this.mountedBikePitch = 0;
    this.mountedSurfacePitch = 0;
    this.mountedSurfaceRoll = 0;
    this.mountedBikeWheelie = 0;
    this.mountedThrottleInput = 0;
    this.mountedTurnInput = 0;
    this.planarVelocity.multiplyScalar(0.35);
    this.targetPlanarVelocity.set(0, 0, 0);
    this.animationRootCompensation.set(0, 0, 0);
    this.animationModeValue = 0;
    this.yaw = bikeYaw;
    this.stopBikeAudio();
  }

  private resolveMountedVelocity(deltaSeconds: number) {
    const turnInput = -this.getMoveRightInput();
    this.mountedTurnInput = turnInput;

    const throttle = this.getMoveForwardInput();
    this.mountedThrottleInput = throttle;
    const cruiseSpeed = Math.max(
      this.sceneSettings.player.runningSpeed + BIKE_CRUISE_SPEED_BONUS,
      this.sceneSettings.player.movementSpeed + 4
    );
    const boostedSpeed = Math.max(cruiseSpeed + 2.5, this.sceneSettings.player.runningSpeed * 1.45);
    const maxSpeed = this.isTouchDevice ? boostedSpeed : this.isRunning() ? boostedSpeed : cruiseSpeed;
    const signedSpeed = throttle >= 0 ? throttle * maxSpeed : throttle * maxSpeed * BIKE_REVERSE_MULTIPLIER;
    const normalizedSpeed = MathUtils.clamp(this.planarVelocity.length() / Math.max(maxSpeed, 0.001), 0, 1);
    const wheelieHeld = (this.keyState.has("Space") || this.mobileWheelieHeld) && throttle > 0;
    const targetWheelie = wheelieHeld ? 0.12 + normalizedSpeed * 0.2 : 0;
    const targetLean = -turnInput * (0.045 + normalizedSpeed * 0.16);
    const targetPitch =
      -targetWheelie + Math.max(throttle, 0) * normalizedSpeed * 0.035 - Math.max(-throttle, 0) * 0.06;

    this.mountedBikeWheelie = damp(this.mountedBikeWheelie, targetWheelie, BIKE_WHEELIE_RESPONSE, deltaSeconds);
    this.mountedBikeLean = damp(this.mountedBikeLean, targetLean, BIKE_ROLL_RESPONSE, deltaSeconds);
    this.mountedBikePitch = damp(
      this.mountedBikePitch,
      targetPitch - this.mountedBikeWheelie,
      BIKE_PITCH_RESPONSE,
      deltaSeconds
    );
  }

  private resolveOnFootVelocity(deltaSeconds: number) {
    const walkSpeed = this.sceneSettings.player.movementSpeed * WALK_SPEED_MULTIPLIER;
    const runSpeed = this.sceneSettings.player.runningSpeed * RUN_SPEED_MULTIPLIER;
    const viewDirection = resolveViewDirection(this.orbitYaw, this.pitch, scratchViewDirection);
    const forward = scratchForward.set(viewDirection.x, 0, viewDirection.z);

    if (forward.lengthSq() > 0) {
      forward.normalize();
    } else {
      forward.set(0, 0, -1);
    }

    const right = scratchRight.set(-forward.z, 0, forward.x).normalize();
    const moveRight = this.getMoveRightInput();
    const moveForward = this.getMoveForwardInput();
    const inputMagnitude = Math.min(Math.hypot(moveRight, moveForward), 1);
    const maxSpeed =
      this.isTouchDevice && this.sceneSettings.player.canRun
        ? runSpeed
        : this.sceneSettings.player.canRun && this.isRunning()
          ? runSpeed
          : walkSpeed;
    this.targetPlanarVelocity
      .set(0, 0, 0)
      .addScaledVector(right, moveRight)
      .addScaledVector(forward, moveForward);

    if (this.targetPlanarVelocity.lengthSq() > 0) {
      this.targetPlanarVelocity.normalize().multiplyScalar(maxSpeed * inputMagnitude);
      this.yaw = dampAngle(this.yaw, yawFromVector(this.targetPlanarVelocity), 14, deltaSeconds);
    }
  }

  private resolvePlanarVelocityRate(grounded: boolean, hasMovementInput: boolean) {
    if (!grounded) {
      return hasMovementInput ? FOOT_AIR_ACCELERATION : FOOT_AIR_DECELERATION;
    }

    return hasMovementInput ? FOOT_ACCELERATION : FOOT_DECELERATION;
  }

  private resolveAnimationSpeedParameter() {
    if (this.mountedBike) {
      return 1;
    }

    const locomotionSpeed = this.planarVelocity.length();
    const walkSpeed = Math.max(this.sceneSettings.player.movementSpeed * WALK_SPEED_MULTIPLIER, 0.001);
    const runSpeed = Math.max(this.sceneSettings.player.runningSpeed * RUN_SPEED_MULTIPLIER, walkSpeed + 0.001);

    if (locomotionSpeed < 0.01) {
      return 0;
    }

    if (locomotionSpeed <= walkSpeed) {
      return MathUtils.clamp(locomotionSpeed / walkSpeed, 0, 1);
    }

    return MathUtils.clamp(1 + (locomotionSpeed - walkSpeed) / (runSpeed - walkSpeed), 1, 2);
  }

  private updateAnimation(deltaSeconds: number) {
    if (!this.animationAnimator || !this.animationDisplayPose || this.animationBonesByName.size === 0) {
      return;
    }

    const nextMode = this.mountedBike ? 2 : 0;

    if (nextMode !== this.animationModeValue) {
      this.animationModeValue = nextMode;
      this.animationRootCompensation.set(0, 0, 0);
    }

    const targetSpeed = this.resolveAnimationSpeedParameter();
    const playbackRate = this.resolveAnimationPlaybackRate(targetSpeed);
    const blendRate = this.mountedBike ? 10 : targetSpeed > 1 ? 8 : 12;
    this.animationSpeedValue = damp(this.animationSpeedValue, targetSpeed, blendRate, deltaSeconds);

    this.animationAnimator.setInt("mode", this.animationModeValue);
    this.animationAnimator.setFloat("speed", this.animationSpeedValue);
    const result = this.animationAnimator.update(deltaSeconds * playbackRate);
    copyPose(result.pose, this.animationDisplayPose);
    forceBoneTranslationToBindPose(
      this.animationDisplayPose.translations,
      this.animationAnimator.rig.bindTranslations,
      this.animationAnimator.rig.rootBoneIndex
    );
    applyPoseBufferToBoneHierarchy(
      this.animationDisplayPose,
      this.animationAnimator.rig.boneNames,
      this.animationBonesByName,
      this.animationSkeletons
    );
    this.applyMountedPoseAdjustments();

    this.animationRootCompensation.x -= result.rootMotion.translation[0] * this.animationScale;
    this.animationRootCompensation.z -= result.rootMotion.translation[2] * this.animationScale;

    if (!this.mountedBike && this.planarVelocity.lengthSq() < 0.0025) {
      this.animationRootCompensation.lerp(scratchZeroVector, 1 - Math.exp(-deltaSeconds * 12));
    }

    this.syncAnimationOffset();
  }

  private resolveAnimationPlaybackRate(targetSpeed: number) {
    const locomotionSpeed = this.planarVelocity.length();

    if (this.mountedBike) {
      return MathUtils.clamp(Math.max(locomotionSpeed, 1) / 2.4, 1, 3.25);
    }

    if (locomotionSpeed < 0.05) {
      return 1;
    }

    // The graph thresholds are 0/1/2, but the controller moves much faster in world units.
    // Scale clip playback separately so footsteps keep up with actual motion.
    return MathUtils.clamp((locomotionSpeed / Math.max(targetSpeed, 1)) * ON_FOOT_ANIMATION_PLAYBACK_SCALE, 1, 6);
  }

  private syncAnimationOffset() {
    if (!this.animationObject) {
      return;
    }

    this.animationObject.position.copy(this.animationBaseOffset).add(this.animationRootCompensation);

    if (this.mountedBike) {
      this.animationObject.position.y += this.mountedBikeWheelie * 0.4;
    }
  }

  private resolveStatusMessage(translation: Vector3) {
    if (this.isTouchDevice) {
      return "";
    }

    if (this.mountedBike) {
      return "Press F to get off the bike";
    }

    if (!this.bike) {
      return "";
    }

    return this.resolveBikeDistance(translation) <= this.bike.mountRadius ? "Press F to ride the bike" : "";
  }

  private resolveGroundHit(translation: Vector3) {
    const hit = raycastClosest({
      direction: { x: 0, y: -1, z: 0 },
      excludeBody: this.body,
      length: GROUND_PROBE_DISTANCE,
      origin: {
        x: translation.x,
        y: translation.y - this.footOffset + GROUND_PROBE_HEIGHT,
        z: translation.z
      },
      world: this.world
    });

    if (!hit || hit.normal.y < GROUND_MIN_NORMAL_Y) {
      return undefined;
    }

    return hit;
  }

  private resolveSurfaceContact(x: number, z: number, startY: number) {
    const hit = raycastClosest({
      direction: { x: 0, y: -1, z: 0 },
      excludeBody: this.body,
      length: 20,
      origin: { x, y: startY, z },
      world: this.world
    });

    if (!hit) {
      return {
        hit: false,
        normal: new Vector3(0, 1, 0),
        position: new Vector3(x, startY, z),
        y: startY
      };
    }

    const y = hit.point.y;
    return {
      hit: true,
      normal: hit.normal.clone(),
      position: hit.point.clone(),
      y
    };
  }

  private setPlayerMountedState(mounted: boolean) {
    rigidBody.setMotionType(this.world, this.body, mounted ? MotionType.KINEMATIC : MotionType.DYNAMIC, true);
    rigidBody.setObjectLayer(
      this.world,
      this.body,
      mounted ? this.world.gameLayers.ghostObjectLayer : this.world.gameLayers.movingObjectLayer
    );
    rigidBody.setLinearVelocity(this.world, this.body, [0, 0, 0]);
    rigidBody.setAngularVelocity(this.world, this.body, [0, 0, 0]);
  }

  private updateBikeAudio() {
    if (!this.mountedBike) {
      this.stopBikeAudio();
      return;
    }

    const throttle = Math.max(this.mountedThrottleInput, 0);
    const speed = this.planarVelocity.length();
    const normalizedSpeed = MathUtils.clamp(speed / 12, 0, 1);
    const targetGain = throttle <= 0.05 ? 0 : MathUtils.clamp(0.08 + throttle * 0.12 + normalizedSpeed * 0.09, 0, 0.29);
    const playbackRate = 0.84 + throttle * 0.2 + normalizedSpeed * 0.45;

    if (targetGain <= 0) {
      this.updateBikeAudioGain(0);
      return;
    }

    void this.ensureBikeAudioPlayback(playbackRate, targetGain);
  }

  private stopBikeAudio() {
    this.updateBikeAudioGain(0);

    if (!this.bikeAudioSource) {
      return;
    }

    this.bikeAudioSource.stop();
    this.bikeAudioSource.disconnect();
    this.bikeAudioSource = null;
  }

  private disposeBikeAudio() {
    this.stopBikeAudio();

    if (!this.bikeAudioContext) {
      return;
    }

    void this.bikeAudioContext.close().catch(() => {});
    this.bikeAudioContext = null;
    this.bikeAudioGainNode = null;
  }

  private updateBikeAudioGain(targetGain: number) {
    if (!this.bikeAudioContext || !this.bikeAudioGainNode) {
      return;
    }

    this.bikeAudioGainNode.gain.setTargetAtTime(targetGain, this.bikeAudioContext.currentTime, 0.08);
  }

  private async ensureBikeAudioPlayback(playbackRate: number, targetGain: number) {
    const graph = this.ensureBikeAudioGraph();

    if (!graph) {
      return;
    }

    if (graph.context.state === "suspended") {
      await graph.context.resume().catch(() => {});
    }

    if (!this.bikeAudioSource) {
      const buffer = await this.loadBikeAudioBuffer(graph.context);

      if (!this.mountedBike) {
        return;
      }

      const source = graph.context.createBufferSource();
      source.buffer = buffer;
      source.loop = true;
      source.connect(graph.gain);
      source.start();
      this.bikeAudioSource = source;
    }

    this.bikeAudioSource.playbackRate.setTargetAtTime(playbackRate, graph.context.currentTime, 0.12);
    graph.gain.gain.setTargetAtTime(targetGain, graph.context.currentTime, 0.08);
  }

  private ensureBikeAudioGraph() {
    if (typeof AudioContext === "undefined") {
      return null;
    }

    if (!this.bikeAudioContext) {
      this.bikeAudioContext = new AudioContext();
    }

    if (!this.bikeAudioGainNode) {
      this.bikeAudioGainNode = this.bikeAudioContext.createGain();
      this.bikeAudioGainNode.gain.value = 0;
      this.bikeAudioGainNode.connect(this.bikeAudioContext.destination);
    }

    return {
      context: this.bikeAudioContext,
      gain: this.bikeAudioGainNode
    };
  }

  private loadBikeAudioBuffer(context: AudioContext) {
    if (!this.bikeAudioBufferPromise) {
      this.bikeAudioBufferPromise = fetch(bikeSoundUrl)
        .then((response) => {
          if (!response.ok) {
            throw new Error(`Failed to load bike sound from ${bikeSoundUrl}`);
          }

          return response.arrayBuffer();
        })
        .then((audioData) => context.decodeAudioData(audioData.slice(0)));
    }

    return this.bikeAudioBufferPromise;
  }

  private applyMountedPoseAdjustments() {
    if (!this.mountedBike) {
      return;
    }

    if (this.mountedHandleBones.length > 0) {
      const handleLeanAmount = this.mountedBikeLean * 0.38;
      const handlePitchAmount = this.mountedBikePitch * 0.12;
      scratchLeanQuaternion.setFromAxisAngle(scratchForwardAxis, handleLeanAmount);
      scratchPitchQuaternion.setFromAxisAngle(scratchRightAxis, handlePitchAmount);

      for (const bone of this.mountedHandleBones) {
        bone.quaternion.multiply(scratchLeanQuaternion).multiply(scratchPitchQuaternion);
        bone.updateMatrix();
      }
    }

    [...this.mountedHandleBones]
      .filter((bone) => !(bone.parent instanceof Bone))
      .forEach((bone) => bone.updateMatrixWorld(true));

    this.animationSkeletons.forEach((skeleton) => skeleton.update());
  }

  private readonly handleCanvasClick = () => {
    if (this.isTouchDevice) {
      return;
    }

    if (document.pointerLockElement === this.domElement) {
      return;
    }

    void this.domElement.requestPointerLock();
  };

  private readonly handleKeyDown = (event: KeyboardEvent) => {
    if (isTextInputTarget(event.target)) {
      return;
    }

    this.keyState.add(event.code);

    if (event.repeat) {
      return;
    }

    if (event.code === "Space") {
      this.jumpQueued = true;
      event.preventDefault();
    }

    if (event.code === BIKE_INTERACT_KEY) {
      this.interactQueued = true;
      event.preventDefault();
    }
  };

  private readonly handleKeyUp = (event: KeyboardEvent) => {
    this.keyState.delete(event.code);
  };

  private readonly handleMouseMove = (event: MouseEvent) => {
    if (this.isTouchDevice) {
      return;
    }

    this.pointerLocked = document.pointerLockElement === this.domElement;

    if (!this.pointerLocked) {
      return;
    }

    this.orbitYaw -= event.movementX * 0.0024;
    this.pitch = MathUtils.clamp(
      this.pitch - event.movementY * 0.0018,
      this.cameraMode === "fps" ? -1.35 : -1.25,
      this.cameraMode === "fps" ? 1.35 : this.cameraMode === "top-down" ? -0.12 : 0.4
    );
  };

  private readonly handleWindowBlur = () => {
    this.keyState.clear();
    this.interactQueued = false;
    this.jumpQueued = false;
    this.mobileMoveX = 0;
    this.mobileMoveY = 0;
    this.mobileOrbitPointerId = null;
    this.mobileWheelieHeld = false;
    this.stopBikeAudio();
    this.releasePointerLock();
  };

  private readonly handleMobileBikeButtonDown = (event: PointerEvent) => {
    event.preventDefault();
    event.stopPropagation();
    this.interactQueued = true;
  };

  private readonly handleMobileWheelieDown = (event: PointerEvent) => {
    event.preventDefault();
    event.stopPropagation();
    this.mobileWheelieHeld = true;
    (event.currentTarget as HTMLElement | null)?.setPointerCapture?.(event.pointerId);
  };

  private readonly handleMobileWheelieUp = (event: PointerEvent) => {
    event.preventDefault();
    event.stopPropagation();
    this.mobileWheelieHeld = false;
    (event.currentTarget as HTMLElement | null)?.releasePointerCapture?.(event.pointerId);
  };

}

function defaultPitchForCameraMode(cameraMode: SceneSettings["player"]["cameraMode"]) {
  if (cameraMode === "fps") {
    return 0;
  }

  if (cameraMode === "third-person") {
    return -0.22;
  }

  return -0.78;
}

function damp(current: number, target: number, rate: number, deltaSeconds: number) {
  return MathUtils.lerp(current, target, 1 - Math.exp(-deltaSeconds * rate));
}

function dampAngle(current: number, target: number, rate: number, deltaSeconds: number) {
  const delta = Math.atan2(Math.sin(target - current), Math.cos(target - current));
  return current + delta * (1 - Math.exp(-deltaSeconds * rate));
}

function isTextInputTarget(target: EventTarget | null) {
  return (
    target instanceof HTMLElement &&
    (target.isContentEditable || target.tagName === "INPUT" || target.tagName === "TEXTAREA")
  );
}

function isLikelyTouchDevice() {
  return (
    typeof window !== "undefined" &&
    ("ontouchstart" in window ||
      navigator.maxTouchPoints > 0 ||
      window.matchMedia?.("(hover: none), (pointer: coarse)").matches === true)
  );
}

function resolveViewDirection(yaw: number, pitch: number, target: Vector3) {
  target.set(
    -Math.sin(yaw) * Math.cos(pitch),
    Math.sin(pitch),
    -Math.cos(yaw) * Math.cos(pitch)
  );

  return target.normalize();
}

function yawFromVector(vector: Vector3) {
  return Math.atan2(-vector.x, -vector.z);
}

const scratchFocusTarget = new Vector3();
const scratchBikeEuler = new Euler(0, 0, 0, "YXZ");
const scratchBikeQuaternion = new Quaternion();
const scratchBodyTranslation = new Vector3();
const scratchCameraLookTarget = new Vector3();
const scratchCurrentVelocity = new Vector3();
const scratchForward = new Vector3();
const scratchForwardAxis = new Vector3(0, 0, 1);
const scratchLeanQuaternion = new Quaternion();
const scratchLocalNormal = new Vector3();
const scratchLinearVelocity = new Vector3();
const scratchMountedBikeQuaternion = new Quaternion();
const scratchMountedBikeOrientation = new Vector3();
const scratchMountedPlanarVelocity = new Vector3();
const scratchMountedRiderPosition = new Vector3();
const scratchPitchQuaternion = new Quaternion();
const scratchRearAnchorOffset = new Vector3();
const scratchRearSample = new Vector3();
const scratchRearPivotWorld = new Vector3();
const scratchRight = new Vector3();
const scratchRightAxis = new Vector3(1, 0, 0);
const scratchRotatedRearAnchor = new Vector3();
const scratchTargetCameraPosition = new Vector3();
const scratchFrontSample = new Vector3();
const scratchInverseYaw = new Quaternion();
const scratchSurfaceNormal = new Vector3();
const scratchViewDirection = new Vector3();
const scratchYawEuler = new Euler(0, 0, 0, "YXZ");
const scratchYawForward = new Vector3();
const scratchYawQuaternion = new Quaternion();
const scratchZeroVector = new Vector3();

function forceBoneTranslationToBindPose(
  translations: Float32Array,
  bindTranslations: Float32Array,
  boneIndex: number
) {
  const offset = boneIndex * 3;
  translations[offset] = bindTranslations[offset]!;
  translations[offset + 1] = bindTranslations[offset + 1]!;
  translations[offset + 2] = bindTranslations[offset + 2]!;
}

function applyPoseBufferToBoneHierarchy(
  pose: PoseBuffer,
  rigBoneNames: readonly string[],
  bonesByName: ReadonlyMap<string, Bone>,
  skeletons: readonly Skeleton[]
) {
  const updatedBones: Bone[] = [];

  rigBoneNames.forEach((boneName, rigBoneIndex) => {
    const bone = bonesByName.get(boneName);

    if (!bone) {
      return;
    }

    const vectorOffset = rigBoneIndex * 3;
    const quaternionOffset = rigBoneIndex * 4;
    bone.position.set(
      pose.translations[vectorOffset]!,
      pose.translations[vectorOffset + 1]!,
      pose.translations[vectorOffset + 2]!
    );
    bone.quaternion.set(
      pose.rotations[quaternionOffset]!,
      pose.rotations[quaternionOffset + 1]!,
      pose.rotations[quaternionOffset + 2]!,
      pose.rotations[quaternionOffset + 3]!
    );
    bone.scale.set(
      pose.scales[vectorOffset]!,
      pose.scales[vectorOffset + 1]!,
      pose.scales[vectorOffset + 2]!
    );
    bone.updateMatrix();
    updatedBones.push(bone);
  });

  updatedBones
    .filter((bone) => !(bone.parent instanceof Bone))
    .forEach((bone) => bone.updateMatrixWorld(true));

  skeletons.forEach((skeleton) => skeleton.update());
}
