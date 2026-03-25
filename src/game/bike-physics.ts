import { box, compound, rigidBody, sphere, MotionQuality, MotionType } from "crashcat";
import { Euler, MathUtils, Matrix4, Quaternion, Vector3, type Object3D } from "three";
import { PhysicsBody, PhysicsWorld, raycastClosest, setQuaternionFromPhysics, setVector3FromPhysics, toPhysicsVec3 } from "./physics";

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

type WheelContact = {
  anchorWorld: Vector3;
  distance: number;
  hit: boolean;
  normal: Vector3;
  point: Vector3;
};

const BIKE_BALANCE_DAMPING = 32;
const BIKE_BALANCE_TORQUE = 220;
const BIKE_DRIVE_FORCE = 1850;
const BIKE_IDLE_GRIP = 170;
const BIKE_LATERAL_GRIP = 310;
const BIKE_MAX_STEER_TORQUE = 180;
const BIKE_REVERSE_FORCE_MULTIPLIER = 0.55;
const BIKE_SEAT_HEIGHT_MULTIPLIER = 1.12;
const BIKE_SUSPENSION_DAMPING = 180;
const BIKE_SUSPENSION_REST = 0.2;
const BIKE_SUSPENSION_STIFFNESS = 1600;
const BIKE_WHEELIE_TORQUE = 95;
const PARKING_BALANCE_MULTIPLIER = 0.72;

export class BikePhysicsRig {
  readonly body: PhysicsBody;
  readonly groundOffset: number;
  readonly mountRadius: number;
  readonly object: Object3D;
  readonly wheelRadius: number;

  private readonly forwardLocal = new Vector3();
  private readonly frontContact = createWheelContact();
  private readonly frontWheelAnchor: Vector3;
  private readonly rearContact = createWheelContact();
  private readonly rearWheelAnchor: Vector3;
  private readonly riderLocalOffset = new Vector3();
  private readonly wheels: Object3D[];
  private readonly world: PhysicsWorld;
  private mounted = false;

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
      .addScaledVector(scratchUp, this.wheelRadius * BIKE_SEAT_HEIGHT_MULTIPLIER);

    this.body = rigidBody.create(options.world, {
      allowSleeping: true,
      angularDamping: 1.8,
      friction: 1.1,
      gravityFactor: 1,
      linearDamping: 0.45,
      mass: 185,
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
  }

  dispose() {
    rigidBody.remove(this.world, this.body);
  }

  getDistanceTo(position: { x: number; z: number }) {
    return Math.hypot(position.x - this.body.position[0]!, position.z - this.body.position[2]!);
  }

  getOrientationEuler(target: Vector3) {
    this.getRiderQuaternion(scratchRiderQuaternion);
    scratchRiderEuler.setFromQuaternion(scratchRiderQuaternion, "YXZ");
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
    const bodyQuaternion = this.getBodyQuaternion(scratchBodyQuaternion);
    const forward = this.getForward(scratchForward, bodyQuaternion);
    const up = this.getAverageUp(scratchUpWorld, bodyQuaternion);
    const right = scratchRight.copy(forward).cross(up).normalize();
    const backward = scratchBackward.copy(forward).multiplyScalar(-1);
    scratchBasis.makeBasis(right, up, backward);
    return target.setFromRotationMatrix(scratchBasis);
  }

  getYaw() {
    const forward = this.getForward(scratchForward, this.getBodyQuaternion(scratchBodyQuaternion));
    return Math.atan2(-forward.x, -forward.z);
  }

  setMounted(mounted: boolean) {
    this.mounted = mounted;
  }

  syncVisuals(deltaSeconds: number) {
    this.object.position.copy(this.getPosition(scratchBodyPosition));
    this.object.quaternion.copy(this.getBodyQuaternion(scratchBodyQuaternion));

    if (this.wheels.length > 0) {
      const forwardSpeed = this.getPlanarVelocity(scratchPlanarVelocity)
        .dot(this.getForward(scratchForward, scratchBodyQuaternion));
      const wheelSpin = forwardSpeed / Math.max(this.wheelRadius, 0.05);

      for (const wheel of this.wheels) {
        wheel.rotateX(-wheelSpin * deltaSeconds);
      }
    }
  }

  updateBeforeStep(deltaSeconds: number, input: BikeControlInput) {
    const bodyQuaternion = this.getBodyQuaternion(scratchBodyQuaternion);
    const bodyPosition = this.getPosition(scratchBodyPosition);
    const bodyVelocity = setVector3FromPhysics(scratchVelocity, this.body.motionProperties.linearVelocity);
    const angularVelocity = setVector3FromPhysics(scratchAngularVelocity, this.body.motionProperties.angularVelocity);
    const forward = this.getForward(scratchForward, bodyQuaternion);
    const up = this.getAverageUp(scratchUpWorld, bodyQuaternion);
    const right = scratchRight.copy(forward).cross(up).normalize();
    const frontContact = this.resolveWheelContact(this.frontWheelAnchor, bodyPosition, bodyQuaternion, up, this.frontContact);
    const rearContact = this.resolveWheelContact(this.rearWheelAnchor, bodyPosition, bodyQuaternion, up, this.rearContact);
    const groundedContacts = [frontContact, rearContact].filter((contact) => contact.hit);
    const groundNormal = groundedContacts.length > 0
      ? groundedContacts.reduce((sum, contact) => sum.add(contact.normal), scratchGroundNormal.set(0, 0, 0)).normalize()
      : scratchGroundNormal.set(0, 1, 0);

    groundedContacts.forEach((contact) => {
      const compression = Math.max(0, this.wheelRadius + BIKE_SUSPENSION_REST - contact.distance);
      const anchorVelocity = setVector3FromPhysics(
        scratchAnchorVelocity,
        rigidBody.getVelocityAtPoint(scratchAnchorVelocityArray, this.body, [contact.anchorWorld.x, contact.anchorWorld.y, contact.anchorWorld.z])
      );
      const springVelocity = anchorVelocity.dot(contact.normal);
      const suspensionForce = Math.max(0, compression * BIKE_SUSPENSION_STIFFNESS - springVelocity * BIKE_SUSPENSION_DAMPING);

      if (suspensionForce > 0) {
        rigidBody.addForceAtPosition(
          this.world,
          this.body,
          toPhysicsVec3(scratchForce.copy(contact.normal).multiplyScalar(suspensionForce)),
          [contact.anchorWorld.x, contact.anchorWorld.y, contact.anchorWorld.z],
          true
        );
      }

      const projectedRight = scratchProjectedRight.copy(right).projectOnPlane(contact.normal).normalize();
      const lateralVelocity = anchorVelocity.dot(projectedRight);
      const gripForce = -(Math.abs(input.throttle) > 0.05 ? BIKE_LATERAL_GRIP : BIKE_IDLE_GRIP) * lateralVelocity;

      if (Number.isFinite(gripForce)) {
        rigidBody.addForceAtPosition(
          this.world,
          this.body,
          toPhysicsVec3(scratchForce.copy(projectedRight).multiplyScalar(gripForce)),
          [contact.anchorWorld.x, contact.anchorWorld.y, contact.anchorWorld.z],
          true
        );
      }
    });

    const driveContact = rearContact.hit ? rearContact : frontContact.hit ? frontContact : undefined;
    const speed = bodyVelocity.dot(forward);

    if (driveContact) {
      const driveDirection = scratchDriveDirection.copy(forward).projectOnPlane(driveContact.normal).normalize();
      const driveForceMagnitude =
        input.throttle >= 0
          ? input.throttle * BIKE_DRIVE_FORCE * (input.boost ? 1.2 : 1)
          : input.throttle * BIKE_DRIVE_FORCE * BIKE_REVERSE_FORCE_MULTIPLIER;

      rigidBody.addForceAtPosition(
        this.world,
        this.body,
        toPhysicsVec3(scratchForce.copy(driveDirection).multiplyScalar(driveForceMagnitude)),
        [driveContact.anchorWorld.x, driveContact.anchorWorld.y, driveContact.anchorWorld.z],
        true
      );
    }

    const speedFactor = MathUtils.clamp(Math.abs(speed) / 10, 0.18, 1);
    const desiredLean = -input.steer * speedFactor * 0.42;
    const desiredUp = scratchDesiredUp.copy(groundNormal).applyAxisAngle(forward, desiredLean);
    const balanceMultiplier = this.mounted ? 1 : PARKING_BALANCE_MULTIPLIER;
    const correctionAxis = scratchCorrectionAxis.copy(up).cross(desiredUp);
    const alignment = 1 - MathUtils.clamp(up.dot(desiredUp), -1, 1);
    const balanceTorque = correctionAxis.multiplyScalar((BIKE_BALANCE_TORQUE + alignment * 120) * balanceMultiplier);
    const angularDamping = scratchAngularDamping.copy(angularVelocity).multiplyScalar(-BIKE_BALANCE_DAMPING * balanceMultiplier);
    const steerTorque = scratchSteerTorque
      .copy(groundNormal)
      .multiplyScalar(input.steer * BIKE_MAX_STEER_TORQUE * speedFactor * Math.sign(speed || input.throttle || 1));

    rigidBody.addTorque(this.world, this.body, toPhysicsVec3(balanceTorque.add(angularDamping).add(steerTorque)), true);

    if (input.wheelie && input.throttle > 0.05) {
      rigidBody.addTorque(
        this.world,
        this.body,
        toPhysicsVec3(scratchWheelieTorque.copy(right).multiplyScalar(-BIKE_WHEELIE_TORQUE)),
        true
      );
    }
  }

  private getAverageUp(target: Vector3, bodyQuaternion: Quaternion) {
    target.copy(scratchUp).applyQuaternion(bodyQuaternion);

    if (!this.frontContact.hit && !this.rearContact.hit) {
      return target.normalize();
    }

    if (this.frontContact.hit && this.rearContact.hit) {
      return target.copy(this.frontContact.normal).add(this.rearContact.normal).normalize();
    }

    return target.copy(this.frontContact.hit ? this.frontContact.normal : this.rearContact.normal).normalize();
  }

  private getBodyQuaternion(target: Quaternion) {
    return setQuaternionFromPhysics(target, this.body.quaternion);
  }

  private getForward(target: Vector3, bodyQuaternion: Quaternion) {
    return target.copy(this.forwardLocal).applyQuaternion(bodyQuaternion).normalize();
  }

  private getPosition(target: Vector3) {
    return setVector3FromPhysics(target, this.body.position);
  }

  private resolveWheelContact(
    anchorLocal: Vector3,
    bodyPosition: Vector3,
    bodyQuaternion: Quaternion,
    up: Vector3,
    output: WheelContact
  ) {
    output.anchorWorld.copy(anchorLocal).applyQuaternion(bodyQuaternion).add(bodyPosition);
    const hit = raycastClosest({
      direction: scratchRayDirection.copy(up).multiplyScalar(-1),
      excludeBody: this.body,
      length: this.wheelRadius + BIKE_SUSPENSION_REST,
      origin: output.anchorWorld,
      world: this.world
    });

    if (!hit) {
      output.distance = this.wheelRadius + BIKE_SUSPENSION_REST;
      output.hit = false;
      output.normal.set(0, 1, 0);
      output.point.copy(output.anchorWorld).addScaledVector(up, -output.distance);
      return output;
    }

    output.distance = hit.distance;
    output.hit = true;
    output.normal.copy(hit.normal);
    output.point.copy(hit.point);
    return output;
  }
}

function createWheelContact(): WheelContact {
  return {
    anchorWorld: new Vector3(),
    distance: 0,
    hit: false,
    normal: new Vector3(0, 1, 0),
    point: new Vector3()
  };
}

const scratchAnchorVelocity = new Vector3();
const scratchAnchorVelocityArray: [number, number, number] = [0, 0, 0];
const scratchAngularDamping = new Vector3();
const scratchAngularVelocity = new Vector3();
const scratchBackward = new Vector3();
const scratchBasis = new Matrix4();
const scratchBodyPosition = new Vector3();
const scratchBodyQuaternion = new Quaternion();
const scratchCorrectionAxis = new Vector3();
const scratchDefaultForward = new Vector3(0, 0, -1);
const scratchDesiredUp = new Vector3();
const scratchDriveDirection = new Vector3();
const scratchForce = new Vector3();
const scratchForward = new Vector3();
const scratchGroundNormal = new Vector3();
const scratchPlanarVelocity = new Vector3();
const scratchProjectedRight = new Vector3();
const scratchRayDirection = new Vector3();
const scratchRight = new Vector3();
const scratchRiderEuler = new Euler(0, 0, 0, "YXZ");
const scratchRiderQuaternion = new Quaternion();
const scratchSteerTorque = new Vector3();
const scratchUp = new Vector3(0, 1, 0);
const scratchUpWorld = new Vector3();
const scratchVelocity = new Vector3();
const scratchWheelbase = new Vector3();
const scratchWheelieTorque = new Vector3();
