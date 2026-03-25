import { box, compound, rigidBody, sphere, MotionQuality, MotionType } from "crashcat";
import { Euler, MathUtils, Matrix4, Quaternion, Vector3, type Object3D } from "three";
import { PhysicsBody, PhysicsWorld, raycastClosest, setQuaternionFromPhysics, setVector3FromPhysics } from "./physics";

export type BikePhysicsRigOptions = {
  frontWheelAnchor: Vector3;
  groundOffset: number;
  mountRadius: number;
  object: Object3D;
  rearWheelAnchor: Vector3;
  wheelRadius: number;
  wheels: Object3D[];
  world: PhysicsWorld;
};

type BikeControlInput = {
  boost: boolean;
  steer: number;
  throttle: number;
  wheelie: boolean;
};

type GroundContact = {
  hit: boolean;
  normal: Vector3;
  point: Vector3;
  supportY: number;
};

type GroundState = {
  centerHit: boolean;
  contactCount: number;
  grounded: boolean;
  normal: Vector3;
  supportY: number;
  wheelContactCount: number;
};

const BIKE_AIR_ANGULAR_DAMPING = 0.4;
const BIKE_AIR_ANGULAR_SPEED_LIMIT = 6.1;
const BIKE_AIR_PITCH_ACCELERATION = 7.9;
const BIKE_AIR_ROLL_ACCELERATION = 1.28;
const BIKE_AIR_YAW_ACCELERATION = 5.05;
const BIKE_CENTER_PROBE_HEIGHT = 0.45;
const BIKE_CENTER_PROBE_MARGIN = 0.55;
const BIKE_COAST_DECELERATION = 5.5;
const BIKE_FORWARD_ACCELERATION = 17.5;
const BIKE_FORWARD_SPEED = 12.8;
const BIKE_BOOST_SPEED = 16.6;
const BIKE_GROUND_CLIMB_RESPONSE = 34;
const BIKE_GROUND_CONTACT_GRACE = 0.08;
const BIKE_GROUND_ALIGN_RESPONSE = 18;
const BIKE_GROUND_ANGULAR_DAMPING = 12;
const BIKE_LAUNCH_ACCELERATION_BOOST = 1.7;
const BIKE_GROUND_NORMAL_MIN_Y = 0.35;
const BIKE_GROUND_NORMAL_RESPONSE = 10;
const BIKE_GROUND_POSITION_RESPONSE = 24;
const BIKE_GROUND_SNAP_DISTANCE = 0.24;
const BIKE_GROUND_UPWARD_STICK_LIMIT = 1.8;
const BIKE_GROUND_VELOCITY_RESPONSE = 10.5;
const BIKE_GROUND_VERTICAL_DAMPING = 18;
const BIKE_LANDING_ANGULAR_WEIGHT = 0.18;
const BIKE_LANDING_AIRTIME_WEIGHT = 0.7;
const BIKE_LANDING_FREE_PHYSICS_MAX = 0.22;
const BIKE_LANDING_FREE_PHYSICS_MIN = 0.05;
const BIKE_LANDING_VERTICAL_WEIGHT = 0.24;
const BIKE_MAX_GROUNDED_PITCH = 0.24;
const BIKE_MAX_GROUNDED_ROLL = 0.42;
const BIKE_MAX_REVERSE_SPEED = 3.8;
const BIKE_OVERSPEED_DRAG = 1.25;
const BIKE_REVERSE_ACCELERATION = 11.5;
const BIKE_REVERSE_DECELERATION = 12.5;
const BIKE_SEAT_HEIGHT_MULTIPLIER = 1.12;
const BIKE_SEAT_VERTICAL_OFFSET = 0.05;
const BIKE_SIDE_FRICTION = 14;
const BIKE_SLOPE_FREE_ROLL_DAMPING_FLAT = 2.1;
const BIKE_SLOPE_FREE_ROLL_DAMPING_STEEP = 0.18;
const BIKE_SLOPE_GRAVITY_ACCELERATION = 1.6;
const BIKE_SLOPE_SIDE_DAMPING_FLAT = BIKE_SIDE_FRICTION;
const BIKE_SLOPE_SIDE_DAMPING_STEEP = 1.1;
const BIKE_STEP_ASSIST_SPEED = 5.6;
const BIKE_STEER_RATE_FAST = 1.85;
const BIKE_STEER_RATE_SLOW = 2.75;
const BIKE_TAKEOFF_ALIGN_RESPONSE = 5.8;
const BIKE_TAKEOFF_ANGULAR_DAMPING = 3.2;
const BIKE_TAKEOFF_CONTACT_RELEASE = 0.34;
const BIKE_TAKEOFF_NORMAL_END_Y = 0.88;
const BIKE_TAKEOFF_NORMAL_START_Y = 0.58;
const BIKE_TAKEOFF_SUPPORT_RELEASE = 0.92;
const BIKE_TAKEOFF_VERTICAL_END = 4.6;
const BIKE_TAKEOFF_VERTICAL_START = 1.4;
const BIKE_TOPPLE_ENTER_TIME = 0.14;
const BIKE_TOPPLE_ANGULAR_DAMPING = 1.35;
const BIKE_TOPPLE_EXIT_TIME = 0.2;
const BIKE_TOPPLE_RECOVERY_ROLL_ACCELERATION = 1.55;
const BIKE_TOPPLE_SIDE_FRICTION = 5.4;
const BIKE_TOPPLE_RECOVERY_SINGLE_CONTACT_UPRIGHT_MIN = 0.72;
const BIKE_TOPPLE_RECOVERY_UPRIGHT_MIN = 0.86;
const BIKE_TOPPLE_SINGLE_CONTACT_UPRIGHT_MIN = 0.34;
const BIKE_TOPPLE_UPRIGHT_MIN = 0.12;
const BIKE_WHEEL_CENTER_SUPPORT_LEAD = 0.08;
const BIKE_WHEEL_PROBE_HEIGHT = 0.55;
const BIKE_WHEEL_PROBE_MARGIN = 0.45;

export class BikePhysicsRig {
  readonly body: PhysicsBody;
  readonly groundOffset: number;
  readonly mountRadius: number;
  readonly object: Object3D;
  readonly wheelRadius: number;

  private readonly centerContact = createGroundContact();
  private readonly forwardLocal = new Vector3();
  private readonly frontContact = createGroundContact();
  private readonly frontWheelAnchor: Vector3;
  private readonly rearContact = createGroundContact();
  private readonly rearWheelAnchor: Vector3;
  private readonly riderLocalOffset = new Vector3();
  private readonly stableGroundNormal = new Vector3(0, 1, 0);
  private readonly wheels: Object3D[];
  private readonly world: PhysicsWorld;
  private airborneTime = 0;
  private groundedGraceRemaining = 0;
  private landingFreePhysicsRemaining = 0;
  private mounted = false;
  private recoveryTime = 0;
  private stableSupportY = 0;
  private toppled = false;
  private toppleTime = 0;
  private wasGrounded = true;
  private yaw = 0;

  constructor(options: BikePhysicsRigOptions) {
    this.world = options.world;
    this.object = options.object;
    this.frontWheelAnchor = options.frontWheelAnchor.clone();
    this.rearWheelAnchor = options.rearWheelAnchor.clone();
    this.groundOffset = options.groundOffset;
    this.mountRadius = options.mountRadius;
    this.wheelRadius = options.wheelRadius;
    this.wheels = options.wheels;

    const wheelbaseVector = scratchWheelbase.copy(this.frontWheelAnchor).sub(this.rearWheelAnchor);
    this.forwardLocal.copy(wheelbaseVector.lengthSq() > 1e-4 ? wheelbaseVector.normalize() : scratchDefaultForward);
    this.riderLocalOffset
      .copy(this.frontWheelAnchor)
      .add(this.rearWheelAnchor)
      .multiplyScalar(0.5)
      .addScaledVector(scratchWorldUp, this.wheelRadius * BIKE_SEAT_HEIGHT_MULTIPLIER + BIKE_SEAT_VERTICAL_OFFSET);

    this.body = rigidBody.create(options.world, {
      allowSleeping: true,
      angularDamping: 1.5,
      friction: 1.25,
      gravityFactor: 1,
      linearDamping: 0.22,
      mass: 165,
      motionQuality: MotionQuality.LINEAR_CAST,
      motionType: MotionType.DYNAMIC,
      objectLayer: options.world.gameLayers.movingObjectLayer,
      position: [options.object.position.x, options.object.position.y, options.object.position.z],
      quaternion: [options.object.quaternion.x, options.object.quaternion.y, options.object.quaternion.z, options.object.quaternion.w],
      restitution: 0,
      shape: compound.create({
        children: [
          {
            position: [this.rearWheelAnchor.x, this.rearWheelAnchor.y, this.rearWheelAnchor.z],
            quaternion: [0, 0, 0, 1],
            shape: sphere.create({ radius: this.wheelRadius * 0.88 })
          },
          {
            position: [this.frontWheelAnchor.x, this.frontWheelAnchor.y, this.frontWheelAnchor.z],
            quaternion: [0, 0, 0, 1],
            shape: sphere.create({ radius: this.wheelRadius * 0.88 })
          },
          {
            position: [
              (this.frontWheelAnchor.x + this.rearWheelAnchor.x) * 0.5,
              (this.frontWheelAnchor.y + this.rearWheelAnchor.y) * 0.5 + this.wheelRadius * 0.45,
              (this.frontWheelAnchor.z + this.rearWheelAnchor.z) * 0.5
            ],
            quaternion: [0, 0, 0, 1],
            shape: box.create({
              convexRadius: 0.03,
              halfExtents: [
                Math.max(0.18, Math.abs(this.frontWheelAnchor.x - this.rearWheelAnchor.x) * 0.15 + 0.1),
                Math.max(0.14, this.wheelRadius * 0.45),
                Math.max(0.24, wheelbaseVector.length() * 0.22)
              ]
            })
          }
        ]
      })
    });

    this.yaw = this.getYaw();
    this.stableSupportY = options.object.position.y;
  }

  dispose() {
    rigidBody.remove(this.world, this.body);
  }

  getDistanceTo(position: { x: number; z: number }) {
    return Math.hypot(position.x - this.body.position[0]!, position.z - this.body.position[2]!);
  }

  getOrientationEuler(target: Vector3) {
    scratchRiderEuler.setFromQuaternion(this.getBodyQuaternion(scratchBodyQuaternion), "YXZ");
    return target.set(scratchRiderEuler.x, scratchRiderEuler.y, scratchRiderEuler.z);
  }

  getPlanarVelocity(target: Vector3) {
    return target.set(
      this.body.motionProperties.linearVelocity[0] ?? 0,
      0,
      this.body.motionProperties.linearVelocity[2] ?? 0
    );
  }

  getRiderPosition(target: Vector3) {
    return target.copy(this.riderLocalOffset).applyQuaternion(this.getBodyQuaternion(scratchBodyQuaternion)).add(this.getPosition(scratchBodyPosition));
  }

  getRiderQuaternion(target: Quaternion) {
    return target.copy(this.getBodyQuaternion(scratchBodyQuaternion));
  }

  getYaw() {
    const forward = this.getForward(scratchForward, this.getBodyQuaternion(scratchBodyQuaternion));
    return Math.atan2(-forward.x, -forward.z);
  }

  resetToUpright() {
    const position = this.getPosition(scratchBodyPosition);
    const yaw = this.getYaw();
    const targetQuaternion = this.composeOrientation(scratchWorldUp, yaw, 0, 0, scratchTargetQuaternion);
    const groundState = this.resolveGroundState(position, targetQuaternion);
    const targetY = groundState.grounded ? groundState.supportY : position.y;

    rigidBody.setTransform(
      this.world,
      this.body,
      [position.x, targetY, position.z],
      [targetQuaternion.x, targetQuaternion.y, targetQuaternion.z, targetQuaternion.w],
      true
    );
    rigidBody.setLinearVelocity(this.world, this.body, [0, 0, 0]);
    rigidBody.setAngularVelocity(this.world, this.body, [0, 0, 0]);
    this.airborneTime = 0;
    this.groundedGraceRemaining = BIKE_GROUND_CONTACT_GRACE;
    this.landingFreePhysicsRemaining = 0;
    this.recoveryTime = 0;
    this.stableGroundNormal.copy(groundState.normal);
    this.stableSupportY = targetY;
    this.toppled = false;
    this.toppleTime = 0;
    this.wasGrounded = true;
    this.yaw = yaw;
  }

  setMounted(mounted: boolean) {
    this.mounted = mounted;
  }

  syncVisuals(deltaSeconds: number) {
    this.object.position.copy(this.getPosition(scratchBodyPosition));
    this.object.quaternion.copy(this.getBodyQuaternion(scratchBodyQuaternion));

    if (this.wheels.length > 0) {
      const forwardSpeed = setVector3FromPhysics(scratchVelocity, this.body.motionProperties.linearVelocity)
        .dot(this.getForward(scratchForward, scratchBodyQuaternion));
      const wheelSpin = forwardSpeed / Math.max(this.wheelRadius, 0.05);

      for (const wheel of this.wheels) {
        wheel.rotateX(-wheelSpin * deltaSeconds);
      }
    }
  }

  updateBeforeStep(deltaSeconds: number, input: BikeControlInput) {
    const position = this.getPosition(scratchBodyPosition);
    const quaternion = this.getBodyQuaternion(scratchBodyQuaternion);
    const velocity = setVector3FromPhysics(scratchVelocity, this.body.motionProperties.linearVelocity);
    const angularVelocity = setVector3FromPhysics(scratchAngularVelocity, this.body.motionProperties.angularVelocity);
    const speedForward = velocity.dot(this.getForward(scratchForward, quaternion));
    const currentUp = scratchUp.set(0, 1, 0).applyQuaternion(quaternion).normalize();

    const currentGroundState = this.resolveStableGroundState(
      deltaSeconds,
      this.resolveGroundState(position, quaternion),
      position,
      velocity
    );

    if (!currentGroundState.grounded) {
      this.airborneTime += deltaSeconds;
      this.recoveryTime = 0;
      this.toppleTime = 0;
      this.wasGrounded = false;
      this.updateAirborne(deltaSeconds, input, quaternion, velocity);
      return;
    }

    if (!this.wasGrounded) {
      const landingImpact = MathUtils.clamp(
        Math.abs(Math.min(velocity.y, 0)) * BIKE_LANDING_VERTICAL_WEIGHT
          + angularVelocity.length() * BIKE_LANDING_ANGULAR_WEIGHT
          + Math.min(this.airborneTime, 1.5) * BIKE_LANDING_AIRTIME_WEIGHT,
        0,
        1
      );
      this.landingFreePhysicsRemaining = Math.max(
        this.landingFreePhysicsRemaining,
        MathUtils.lerp(BIKE_LANDING_FREE_PHYSICS_MIN, BIKE_LANDING_FREE_PHYSICS_MAX, landingImpact)
      );
    } else {
      this.landingFreePhysicsRemaining = Math.max(0, this.landingFreePhysicsRemaining - deltaSeconds);
    }

    this.airborneTime = 0;
    this.wasGrounded = true;
    const bodyYaw = this.getYaw();
    const throttleInput = input.throttle;
    const steerInput = input.steer;
    const rideableGroundState = this.isRideableGroundState(currentGroundState, currentUp);
    const recoveredGroundState = this.isRecoveredGroundState(currentGroundState, currentUp);

    if (this.toppled) {
      this.toppleTime = 0;
      this.recoveryTime = recoveredGroundState ? this.recoveryTime + deltaSeconds : 0;

      if (this.recoveryTime >= BIKE_TOPPLE_EXIT_TIME) {
        this.toppled = false;
        this.recoveryTime = 0;
      }
    } else {
      this.recoveryTime = 0;
      this.toppleTime = rideableGroundState ? Math.max(0, this.toppleTime - deltaSeconds * 2) : this.toppleTime + deltaSeconds;

      if (this.toppleTime >= BIKE_TOPPLE_ENTER_TIME) {
        this.toppled = true;
      }
    }

    if (this.toppled) {
      this.landingFreePhysicsRemaining = 0;
      this.yaw = bodyYaw;
      this.updateToppledGrounded(deltaSeconds, input, quaternion, velocity, angularVelocity, currentGroundState);
      return;
    }

    if (this.landingFreePhysicsRemaining > 0) {
      this.yaw = bodyYaw;
      return;
    }

    const speedNormalized = MathUtils.clamp(Math.abs(speedForward) / BIKE_BOOST_SPEED, 0, 1);
    const steerRate = MathUtils.lerp(BIKE_STEER_RATE_SLOW, BIKE_STEER_RATE_FAST, speedNormalized);
    const takeoffFactor = this.resolveTakeoffFactor(currentGroundState, velocity, speedForward);
    const steerDirection =
      speedForward < -0.15 || (Math.abs(speedForward) < 0.15 && throttleInput < -0.05)
        ? -1
        : 1;

    if (Math.abs(steerInput) > 0.001) {
      this.yaw += steerInput * steerRate * deltaSeconds * steerDirection;
    }

    const targetYaw = this.yaw;

    const groundedPitch = MathUtils.clamp(
      -Math.max(input.throttle, 0) * 0.05 * (0.35 + speedNormalized * 0.65)
        + Math.max(-input.throttle, 0) * 0.1
        - ((input.wheelie && input.throttle > 0) ? 0.22 + speedNormalized * 0.16 : 0),
      -BIKE_MAX_GROUNDED_PITCH,
      BIKE_MAX_GROUNDED_PITCH * 1.75
    );
    const groundedRoll = MathUtils.clamp(
      -input.steer * (0.12 + speedNormalized * 0.3),
      -BIKE_MAX_GROUNDED_ROLL,
      BIKE_MAX_GROUNDED_ROLL
    );

    const launchGroundNormal = scratchLaunchGroundNormal
      .copy(currentGroundState.normal)
      .lerp(currentUp, takeoffFactor)
      .normalize();
    const roughQuaternion = this.composeOrientation(
      launchGroundNormal,
      targetYaw,
      groundedRoll,
      groundedPitch,
      scratchTargetQuaternion
    );
    const refinedGroundState = this.resolveStableGroundState(
      deltaSeconds,
      this.resolveGroundState(position, roughQuaternion),
      position,
      velocity
    );
    const refinedLaunchNormal = scratchRefinedLaunchNormal
      .copy(refinedGroundState.normal)
      .lerp(currentUp, takeoffFactor)
      .normalize();
    const targetQuaternion = this.composeOrientation(
      refinedLaunchNormal,
      targetYaw,
      groundedRoll,
      groundedPitch,
      scratchTargetQuaternion
    );
    const alignResponse = MathUtils.lerp(BIKE_GROUND_ALIGN_RESPONSE, BIKE_TAKEOFF_ALIGN_RESPONSE, takeoffFactor);
    const rotationAlpha = 1 - Math.exp(-deltaSeconds * alignResponse);
    const smoothedQuaternion = scratchSmoothedQuaternion.copy(quaternion).slerp(targetQuaternion, rotationAlpha);

    const desiredForward = this.getForward(scratchDesiredForward, smoothedQuaternion)
      .projectOnPlane(refinedGroundState.normal)
      .normalize();

    if (desiredForward.lengthSq() < 1e-4) {
      desiredForward.copy(this.getHeadingForward(scratchHeadingForward, this.yaw));
    }

    const launchForward = scratchLaunchForward.copy(desiredForward);

    const driveForward = scratchDriveForward.copy(desiredForward).setY(0);

    if (driveForward.lengthSq() < 1e-4) {
      driveForward.copy(this.getHeadingForward(scratchHeadingForward, this.yaw));
    } else {
      driveForward.normalize();
    }

    const driveRight = scratchDriveRight.set(-driveForward.z, 0, driveForward.x).normalize();
    const planarVelocity = scratchPlanarVelocity.copy(velocity).setY(0);
    const launchForwardSpeed = velocity.dot(launchForward);
    const currentForwardSpeed = MathUtils.lerp(planarVelocity.dot(driveForward), launchForwardSpeed, takeoffFactor);
    const currentSideSpeed = planarVelocity.dot(driveRight);
    const slopeAcceleration = this.resolveSlopeAcceleration(refinedGroundState.normal);
    const steepness = MathUtils.clamp(1 - refinedGroundState.normal.y, 0, 1);
    const idleThrottle = Math.abs(throttleInput) < 0.05;
    let nextForwardSpeed: number;
    let nextSideSpeed: number;

    if (idleThrottle) {
      const rolledPlanarVelocity = scratchRolledPlanarVelocity.copy(planarVelocity).addScaledVector(slopeAcceleration, deltaSeconds);
      const forwardDamping = MathUtils.lerp(
        BIKE_COAST_DECELERATION,
        BIKE_SLOPE_FREE_ROLL_DAMPING_STEEP,
        steepness
      );
      const sideDamping = MathUtils.lerp(BIKE_SLOPE_SIDE_DAMPING_FLAT, BIKE_SLOPE_SIDE_DAMPING_STEEP, steepness);
      nextForwardSpeed = damp(rolledPlanarVelocity.dot(driveForward), 0, forwardDamping, deltaSeconds);
      nextSideSpeed = damp(rolledPlanarVelocity.dot(driveRight), 0, sideDamping, deltaSeconds);
    } else {
      const slopeForwardDelta = slopeAcceleration.dot(driveForward) * deltaSeconds;
      const slopeSideDelta = slopeAcceleration.dot(driveRight) * deltaSeconds;
      const launchAccelerationMultiplier = MathUtils.lerp(BIKE_LAUNCH_ACCELERATION_BOOST, 1, speedNormalized);
      const forwardAcceleration =
        (throttleInput >= 0 ? BIKE_FORWARD_ACCELERATION : BIKE_REVERSE_ACCELERATION) * launchAccelerationMultiplier;
      nextForwardSpeed = currentForwardSpeed + throttleInput * forwardAcceleration * deltaSeconds + slopeForwardDelta;

      if (currentForwardSpeed * throttleInput < 0) {
        nextForwardSpeed = moveTowards(nextForwardSpeed, 0, BIKE_REVERSE_DECELERATION * deltaSeconds);
      }

      const forwardSpeedCap = input.boost ? BIKE_BOOST_SPEED : BIKE_FORWARD_SPEED;

      if (nextForwardSpeed > forwardSpeedCap) {
        nextForwardSpeed -= Math.min(nextForwardSpeed - forwardSpeedCap, BIKE_OVERSPEED_DRAG * deltaSeconds);
      }

      if (nextForwardSpeed < -BIKE_MAX_REVERSE_SPEED) {
        nextForwardSpeed += Math.min(-BIKE_MAX_REVERSE_SPEED - nextForwardSpeed, BIKE_OVERSPEED_DRAG * deltaSeconds);
      }

      nextSideSpeed =
        damp(currentSideSpeed, 0, BIKE_SIDE_FRICTION, deltaSeconds)
        + slopeSideDelta;
    }

    const launchVelocityDirection = scratchLaunchVelocityDirection
      .copy(driveForward)
      .lerp(launchForward, takeoffFactor)
      .normalize();
    const desiredVelocity = scratchDesiredVelocity
      .copy(launchVelocityDirection)
      .multiplyScalar(nextForwardSpeed)
      .addScaledVector(driveRight, nextSideSpeed);
    const velocityResponse = MathUtils.lerp(BIKE_GROUND_VELOCITY_RESPONSE, 4.8, takeoffFactor);
    const smoothedPlanarVelocity = scratchSmoothedPlanarVelocity
      .copy(planarVelocity)
      .lerp(scratchTargetPlanarVelocity.copy(desiredVelocity).setY(0), 1 - Math.exp(-deltaSeconds * velocityResponse));

    const supportResponseBase = refinedGroundState.supportY > position.y
      ? BIKE_GROUND_CLIMB_RESPONSE
      : BIKE_GROUND_POSITION_RESPONSE;
    const supportResponse = MathUtils.lerp(supportResponseBase, BIKE_GROUND_POSITION_RESPONSE, takeoffFactor);
    const supportY = MathUtils.lerp(
      damp(position.y, refinedGroundState.supportY, supportResponse, deltaSeconds),
      position.y,
      takeoffFactor * BIKE_TAKEOFF_SUPPORT_RELEASE
    );

    rigidBody.setTransform(
      this.world,
      this.body,
      [position.x, supportY, position.z],
      [smoothedQuaternion.x, smoothedQuaternion.y, smoothedQuaternion.z, smoothedQuaternion.w],
      true
    );
    const climbDelta = Math.max(
      0,
      refinedGroundState.supportY - position.y,
      this.frontContact.hit ? this.frontContact.supportY - position.y : 0,
      this.centerContact.hit ? this.centerContact.supportY - position.y : 0
    );
    const climbVelocity =
      input.throttle > 0.05 && climbDelta > 0.015
        ? Math.min(BIKE_STEP_ASSIST_SPEED, climbDelta / Math.max(deltaSeconds, 1e-3))
        : 0;
    const groundedVerticalVelocity = MathUtils.lerp(
      damp(velocity.y, 0, BIKE_GROUND_VERTICAL_DAMPING, deltaSeconds),
      velocity.y,
      takeoffFactor
    );

    rigidBody.setLinearVelocity(this.world, this.body, [
      smoothedPlanarVelocity.x,
      Math.max(climbVelocity, groundedVerticalVelocity, desiredVelocity.y),
      smoothedPlanarVelocity.z
    ]);
    const angularDamping = MathUtils.lerp(BIKE_GROUND_ANGULAR_DAMPING, BIKE_TAKEOFF_ANGULAR_DAMPING, takeoffFactor);
    const dampedAngularVelocity = scratchGroundAngularVelocity.copy(angularVelocity).multiplyScalar(
      Math.exp(-angularDamping * deltaSeconds)
    );
    rigidBody.setAngularVelocity(this.world, this.body, [
      dampedAngularVelocity.x,
      MathUtils.lerp(dampedAngularVelocity.y, 0, 1 - Math.exp(-deltaSeconds * (angularDamping * 0.75))),
      dampedAngularVelocity.z
    ]);
  }

  private composeOrientation(groundNormal: Vector3, yaw: number, roll: number, pitch: number, target: Quaternion) {
    const forward = this.getHeadingForward(scratchHeadingForward, yaw)
      .projectOnPlane(groundNormal)
      .normalize();

    if (forward.lengthSq() < 1e-4) {
      forward.copy(scratchDefaultForward);
    }

    const up = scratchDesiredUp.copy(groundNormal).normalize().applyAxisAngle(forward, roll);
    const right = scratchDesiredRight.copy(forward).cross(up).normalize();

    if (pitch !== 0) {
      forward.applyAxisAngle(right, pitch).normalize();
      up.applyAxisAngle(right, pitch).normalize();
    }

    right.crossVectors(forward, up).normalize();
    up.crossVectors(right, forward).normalize();
    scratchBasis.makeBasis(right, up, scratchBackward.copy(forward).multiplyScalar(-1));
    return target.setFromRotationMatrix(scratchBasis);
  }

  private getBodyQuaternion(target: Quaternion) {
    return setQuaternionFromPhysics(target, this.body.quaternion);
  }

  private getForward(target: Vector3, bodyQuaternion: Quaternion) {
    return target.copy(this.forwardLocal).applyQuaternion(bodyQuaternion).normalize();
  }

  private getHeadingForward(target: Vector3, yaw: number) {
    return target.set(-Math.sin(yaw), 0, -Math.cos(yaw)).normalize();
  }

  private getPosition(target: Vector3) {
    return setVector3FromPhysics(target, this.body.position);
  }

  private resolveSlopeAcceleration(groundNormal: Vector3) {
    const gravity = scratchGravity.set(
      this.world.settings.gravity[0] ?? 0,
      this.world.settings.gravity[1] ?? -9.81,
      this.world.settings.gravity[2] ?? 0
    );
    const slopeVector = scratchSlopeVector.copy(gravity).projectOnPlane(groundNormal).setY(0);

    if (slopeVector.lengthSq() < 1e-5) {
      return slopeVector.set(0, 0, 0);
    }

    const steepness = MathUtils.clamp(1 - groundNormal.y, 0, 1);
    return slopeVector.normalize().multiplyScalar(gravity.length() * steepness * BIKE_SLOPE_GRAVITY_ACCELERATION);
  }

  private resolveTakeoffFactor(groundState: GroundState, velocity: Vector3, speedForward: number) {
    const speedWeight = MathUtils.smoothstep(Math.abs(speedForward), BIKE_FORWARD_SPEED * 0.45, BIKE_BOOST_SPEED * 0.9);
    const steepnessRelease = 1 - MathUtils.smoothstep(groundState.normal.y, BIKE_TAKEOFF_NORMAL_START_Y, BIKE_TAKEOFF_NORMAL_END_Y);
    const contactRelease = groundState.wheelContactCount < 2 ? BIKE_TAKEOFF_CONTACT_RELEASE : 0;
    const verticalRelease = MathUtils.smoothstep(Math.max(velocity.y, 0), BIKE_TAKEOFF_VERTICAL_START, BIKE_TAKEOFF_VERTICAL_END) * 0.42;
    return MathUtils.clamp(steepnessRelease * speedWeight + contactRelease * speedWeight + verticalRelease, 0, 0.92);
  }

  private isRideableGroundState(groundState: GroundState, currentUp: Vector3) {
    if (currentUp.y < BIKE_TOPPLE_UPRIGHT_MIN) {
      return false;
    }

    if (groundState.wheelContactCount < 2 && currentUp.y < BIKE_TOPPLE_SINGLE_CONTACT_UPRIGHT_MIN) {
      return false;
    }

    return true;
  }

  private isRecoveredGroundState(groundState: GroundState, currentUp: Vector3) {
    if (currentUp.y < BIKE_TOPPLE_RECOVERY_UPRIGHT_MIN) {
      return false;
    }

    if (groundState.wheelContactCount < 2 && currentUp.y < BIKE_TOPPLE_RECOVERY_SINGLE_CONTACT_UPRIGHT_MIN) {
      return false;
    }

    return true;
  }

  private resolveStableGroundState(
    deltaSeconds: number,
    groundState: GroundState,
    position: Vector3,
    velocity: Vector3
  ): GroundState {
    if (groundState.grounded) {
      const normalAlpha = 1 - Math.exp(-deltaSeconds * BIKE_GROUND_NORMAL_RESPONSE);

      if (this.groundedGraceRemaining <= 0) {
        this.stableGroundNormal.copy(groundState.normal);
      } else {
        this.stableGroundNormal.lerp(groundState.normal, normalAlpha).normalize();
      }

      this.stableSupportY = groundState.supportY;
      this.groundedGraceRemaining = BIKE_GROUND_CONTACT_GRACE;
      return {
        centerHit: groundState.centerHit,
        contactCount: groundState.contactCount,
        grounded: true,
        normal: scratchStableGroundNormal.copy(this.stableGroundNormal),
        supportY: this.stableSupportY,
        wheelContactCount: groundState.wheelContactCount
      };
    }

    const withinSnapDistance = position.y - this.stableSupportY <= BIKE_GROUND_SNAP_DISTANCE;

    if (
      this.groundedGraceRemaining > 0
      && velocity.y <= BIKE_GROUND_UPWARD_STICK_LIMIT
      && withinSnapDistance
    ) {
      this.groundedGraceRemaining = Math.max(0, this.groundedGraceRemaining - deltaSeconds);
      return {
        centerHit: false,
        contactCount: 1,
        grounded: true,
        normal: scratchStableGroundNormal.copy(this.stableGroundNormal),
        supportY: this.stableSupportY,
        wheelContactCount: 1
      };
    }

    this.groundedGraceRemaining = 0;
    return groundState;
  }

  private updateToppledGrounded(
    deltaSeconds: number,
    input: BikeControlInput,
    quaternion: Quaternion,
    velocity: Vector3,
    angularVelocity: Vector3,
    groundState: GroundState
  ) {
    if (!this.mounted) {
      this.yaw = this.getYaw();
      return;
    }

    const toppledForward = this.getForward(scratchDesiredForward, quaternion).projectOnPlane(groundState.normal);

    if (toppledForward.lengthSq() < 1e-4) {
      toppledForward.copy(this.getHeadingForward(scratchHeadingForward, this.yaw));
    } else {
      toppledForward.normalize();
    }

    const toppledRight = scratchDesiredRight.set(-toppledForward.z, 0, toppledForward.x);

    if (toppledRight.lengthSq() < 1e-4) {
      toppledRight.set(1, 0, 0);
    } else {
      toppledRight.normalize();
    }

    const planarVelocity = scratchPlanarVelocity.copy(velocity).setY(0);
    const currentForwardSpeed = planarVelocity.dot(toppledForward);
    const currentSideSpeed = planarVelocity.dot(toppledRight);
    const speedNormalized = MathUtils.clamp(Math.abs(currentForwardSpeed) / BIKE_BOOST_SPEED, 0, 1);
    const slopeAcceleration = this.resolveSlopeAcceleration(groundState.normal);
    const slopeForwardDelta = slopeAcceleration.dot(toppledForward) * deltaSeconds;
    const slopeSideDelta = slopeAcceleration.dot(toppledRight) * deltaSeconds;
    const launchAccelerationMultiplier = MathUtils.lerp(BIKE_LAUNCH_ACCELERATION_BOOST, 1, speedNormalized);
    const forwardAcceleration =
      (input.throttle >= 0 ? BIKE_FORWARD_ACCELERATION * 0.72 : BIKE_REVERSE_ACCELERATION * 0.62)
      * launchAccelerationMultiplier;
    let nextForwardSpeed = currentForwardSpeed + input.throttle * forwardAcceleration * deltaSeconds + slopeForwardDelta;
    const forwardSpeedCap = input.boost ? BIKE_BOOST_SPEED : BIKE_FORWARD_SPEED;

    if (nextForwardSpeed > forwardSpeedCap) {
      nextForwardSpeed -= Math.min(nextForwardSpeed - forwardSpeedCap, BIKE_OVERSPEED_DRAG * deltaSeconds);
    }

    if (nextForwardSpeed < -BIKE_MAX_REVERSE_SPEED) {
      nextForwardSpeed += Math.min(-BIKE_MAX_REVERSE_SPEED - nextForwardSpeed, BIKE_OVERSPEED_DRAG * deltaSeconds);
    }

    const nextSideSpeed = damp(currentSideSpeed + slopeSideDelta, 0, BIKE_TOPPLE_SIDE_FRICTION, deltaSeconds);
    const desiredVelocity = scratchDesiredVelocity
      .copy(toppledForward)
      .multiplyScalar(nextForwardSpeed)
      .addScaledVector(toppledRight, nextSideSpeed);

    rigidBody.setLinearVelocity(this.world, this.body, [desiredVelocity.x, velocity.y, desiredVelocity.z]);

    const toppledAngularVelocity = scratchGroundAngularVelocity.copy(angularVelocity).multiplyScalar(
      Math.exp(-BIKE_TOPPLE_ANGULAR_DAMPING * deltaSeconds)
    );

    if (Math.abs(input.steer) > 0.02) {
      toppledAngularVelocity.addScaledVector(
        toppledForward,
        -input.steer * BIKE_TOPPLE_RECOVERY_ROLL_ACCELERATION * deltaSeconds
      );
    }

    rigidBody.setAngularVelocity(this.world, this.body, [
      toppledAngularVelocity.x,
      toppledAngularVelocity.y,
      toppledAngularVelocity.z
    ]);
  }

  private resolveCenterSupport(bodyPosition: Vector3) {
    const hit = raycastClosest({
      direction: scratchWorldDown,
      excludeBody: this.body,
      excludeObjectLayers: [this.world.gameLayers.ghostObjectLayer],
      length: BIKE_CENTER_PROBE_HEIGHT + this.groundOffset + BIKE_CENTER_PROBE_MARGIN,
      origin: {
        x: bodyPosition.x,
        y: bodyPosition.y + BIKE_CENTER_PROBE_HEIGHT,
        z: bodyPosition.z
      },
      world: this.world
    });

    if (!hit || hit.normal.y < BIKE_GROUND_NORMAL_MIN_Y) {
      this.centerContact.hit = false;
      this.centerContact.normal.copy(scratchWorldUp);
      this.centerContact.point.set(bodyPosition.x, bodyPosition.y - this.groundOffset, bodyPosition.z);
      this.centerContact.supportY = bodyPosition.y;
      return this.centerContact;
    }

    this.centerContact.hit = true;
    this.centerContact.normal.copy(hit.normal);
    this.centerContact.point.copy(hit.point);
    this.centerContact.supportY = hit.point.y + this.groundOffset + 0.02;
    return this.centerContact;
  }

  private resolveGroundState(bodyPosition: Vector3, orientation: Quaternion): GroundState {
    const frontContact = this.resolveWheelContact(this.frontWheelAnchor, bodyPosition, orientation, this.frontContact);
    const rearContact = this.resolveWheelContact(this.rearWheelAnchor, bodyPosition, orientation, this.rearContact);
    const centerContact = this.resolveCenterSupport(bodyPosition);

    let accumulatedSupportY = 0;
    let contactCount = 0;
    let highestSupportY = Number.NEGATIVE_INFINITY;
    let wheelContactCount = 0;
    scratchGroundNormal.set(0, 0, 0);

    for (const contact of [frontContact, rearContact]) {
      if (!contact.hit) {
        continue;
      }

      contactCount += 1;
      wheelContactCount += 1;
      accumulatedSupportY += contact.supportY;
      highestSupportY = Math.max(highestSupportY, contact.supportY);
      scratchGroundNormal.add(contact.normal);
    }

    if (wheelContactCount === 0) {
      if (centerContact.hit) {
        return {
          contactCount: 1,
          centerHit: true,
          grounded: true,
          normal: scratchGroundNormal.copy(centerContact.normal),
          supportY: centerContact.supportY,
          wheelContactCount: 0
        };
      }

      return {
        contactCount: 0,
        centerHit: false,
        grounded: false,
        normal: scratchGroundNormal.copy(scratchWorldUp),
        supportY: bodyPosition.y,
        wheelContactCount: 0
      };
    }

    const averagedSupportY = accumulatedSupportY / wheelContactCount;
    const steppedSupportY = Math.max(averagedSupportY, highestSupportY - 0.03);
    const centerSupportY = centerContact.hit
      ? Math.min(centerContact.supportY, steppedSupportY + BIKE_WHEEL_CENTER_SUPPORT_LEAD)
      : steppedSupportY;

    if (centerContact.hit) {
      contactCount += 1;
      scratchGroundNormal.addScaledVector(centerContact.normal, wheelContactCount < 2 ? 0.55 : 0.35);
    }

    return {
      contactCount,
      centerHit: centerContact.hit,
      grounded: true,
      normal: scratchGroundNormal.normalize(),
      supportY: Math.max(steppedSupportY, centerSupportY),
      wheelContactCount
    };
  }

  private resolveWheelContact(anchorLocal: Vector3, bodyPosition: Vector3, orientation: Quaternion, output: GroundContact) {
    const rotatedAnchor = scratchRotatedAnchor.copy(anchorLocal).applyQuaternion(orientation);
    const anchorWorld = scratchAnchorWorld.copy(bodyPosition).add(rotatedAnchor);
    const hit = raycastClosest({
      direction: scratchWorldDown,
      excludeBody: this.body,
      excludeObjectLayers: [this.world.gameLayers.ghostObjectLayer],
      length: BIKE_WHEEL_PROBE_HEIGHT + this.wheelRadius + this.groundOffset + BIKE_WHEEL_PROBE_MARGIN,
      origin: {
        x: anchorWorld.x,
        y: anchorWorld.y + BIKE_WHEEL_PROBE_HEIGHT,
        z: anchorWorld.z
      },
      world: this.world
    });

    if (!hit || hit.normal.y < BIKE_GROUND_NORMAL_MIN_Y) {
      output.hit = false;
      output.normal.copy(scratchWorldUp);
      output.point.copy(anchorWorld).addScaledVector(scratchWorldDown, this.wheelRadius);
      output.supportY = bodyPosition.y;
      return output;
    }

    output.hit = true;
    output.normal.copy(hit.normal);
    output.point.copy(hit.point);
    output.supportY = hit.point.y + hit.normal.y * this.wheelRadius - rotatedAnchor.y;
    return output;
  }

  private updateAirborne(deltaSeconds: number, input: BikeControlInput, quaternion: Quaternion, velocity: Vector3) {
    const forward = this.getForward(scratchForward, quaternion);
    const right = scratchAirRight.set(1, 0, 0).applyQuaternion(quaternion).normalize();
    const up = scratchAirUp.set(0, 1, 0).applyQuaternion(quaternion).normalize();
    const throttlePitch = input.throttle;
    const wheeliePitch = input.wheelie && input.throttle > 0 ? 0.95 : 0;
    const pitchAcceleration = -MathUtils.clamp(throttlePitch + wheeliePitch, -1, 1) * BIKE_AIR_PITCH_ACCELERATION;
    const rollAcceleration = -input.steer * BIKE_AIR_ROLL_ACCELERATION;
    const yawAcceleration = input.steer * BIKE_AIR_YAW_ACCELERATION;

    const currentAngularVelocity = setVector3FromPhysics(scratchAngularVelocity, this.body.motionProperties.angularVelocity);
    currentAngularVelocity
      .addScaledVector(right, pitchAcceleration * deltaSeconds)
      .addScaledVector(forward, rollAcceleration * deltaSeconds)
      .addScaledVector(up, yawAcceleration * deltaSeconds)
      .multiplyScalar(Math.exp(-BIKE_AIR_ANGULAR_DAMPING * deltaSeconds));

    if (currentAngularVelocity.length() > BIKE_AIR_ANGULAR_SPEED_LIMIT) {
      currentAngularVelocity.setLength(BIKE_AIR_ANGULAR_SPEED_LIMIT);
    }

    rigidBody.setLinearVelocity(this.world, this.body, [velocity.x, velocity.y, velocity.z]);
    rigidBody.setAngularVelocity(this.world, this.body, [
      currentAngularVelocity.x,
      currentAngularVelocity.y,
      currentAngularVelocity.z
    ]);
    this.yaw = this.getYaw();
  }
}

function createGroundContact(): GroundContact {
  return {
    hit: false,
    normal: new Vector3(0, 1, 0),
    point: new Vector3(),
    supportY: 0
  };
}

function damp(current: number, target: number, smoothing: number, deltaSeconds: number) {
  return MathUtils.lerp(current, target, 1 - Math.exp(-smoothing * deltaSeconds));
}

function dampAngle(current: number, target: number, rate: number, deltaSeconds: number) {
  const delta = Math.atan2(Math.sin(target - current), Math.cos(target - current));
  return current + delta * (1 - Math.exp(-deltaSeconds * rate));
}

function moveTowards(current: number, target: number, maxDelta: number) {
  if (Math.abs(target - current) <= maxDelta) {
    return target;
  }

  return current + Math.sign(target - current) * maxDelta;
}

const scratchAnchorWorld = new Vector3();
const scratchAirRight = new Vector3();
const scratchAirUp = new Vector3();
const scratchAngularVelocity = new Vector3();
const scratchBackward = new Vector3();
const scratchBasis = new Matrix4();
const scratchBodyPosition = new Vector3();
const scratchBodyQuaternion = new Quaternion();
const scratchDefaultForward = new Vector3(0, 0, -1);
const scratchDriveForward = new Vector3();
const scratchDriveRight = new Vector3();
const scratchDesiredForward = new Vector3();
const scratchDesiredRight = new Vector3();
const scratchDesiredUp = new Vector3();
const scratchDesiredVelocity = new Vector3();
const scratchForward = new Vector3();
const scratchGravity = new Vector3();
const scratchGroundAngularVelocity = new Vector3();
const scratchGroundNormal = new Vector3();
const scratchHeadingForward = new Vector3();
const scratchLaunchGroundNormal = new Vector3();
const scratchLaunchForward = new Vector3();
const scratchLaunchVelocityDirection = new Vector3();
const scratchPlanarVelocity = new Vector3();
const scratchRefinedLaunchNormal = new Vector3();
const scratchRight = new Vector3();
const scratchRolledPlanarVelocity = new Vector3();
const scratchRiderEuler = new Euler(0, 0, 0, "YXZ");
const scratchRotatedAnchor = new Vector3();
const scratchSmoothedQuaternion = new Quaternion();
const scratchSmoothedPlanarVelocity = new Vector3();
const scratchSlopeVector = new Vector3();
const scratchStableGroundNormal = new Vector3();
const scratchTargetQuaternion = new Quaternion();
const scratchTargetPlanarVelocity = new Vector3();
const scratchUp = new Vector3();
const scratchVelocity = new Vector3();
const scratchWheelbase = new Vector3();
const scratchWorldDown = new Vector3(0, -1, 0);
const scratchWorldUp = new Vector3(0, 1, 0);
