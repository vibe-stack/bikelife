import { DirectionalLight, PCFSoftShadowMap, type Object3D, type WebGLRenderer } from "three";

type ShadowCandidate = Object3D & {
  castShadow?: boolean;
  frustumCulled?: boolean;
  receiveShadow?: boolean;
};

export function configureRendererShadows(renderer: WebGLRenderer) {
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = PCFSoftShadowMap;
}

export function configureDirectionalShadowLight(light: DirectionalLight) {
  light.castShadow = true;
  light.shadow.mapSize.set(2048, 2048);
  light.shadow.bias = -0.00012;
  light.shadow.normalBias = 0.03;
  light.shadow.camera.near = 1;
  light.shadow.camera.far = 180;
  light.shadow.camera.left = -90;
  light.shadow.camera.right = 90;
  light.shadow.camera.top = 90;
  light.shadow.camera.bottom = -90;
  light.shadow.camera.updateProjectionMatrix();
}

export function applyShadowFlags(root: Object3D) {
  root.traverse((child) => {
    const candidate = child as ShadowCandidate;

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
}
