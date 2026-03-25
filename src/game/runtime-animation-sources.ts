import type { AnimationClipAsset, RigDefinition } from "@ggez/anim-core";
import { loadClipsFromArtifact, loadRigFromArtifact, parseAnimationArtifactJson } from "@ggez/anim-exporter";
import { parseAnimationBundle, type AnimationArtifact, type AnimationBundle } from "@ggez/anim-schema";
import { createClipAssetFromThreeClip, createRigFromSkeleton } from "@ggez/anim-three";
import type { AnimationClip, Object3D, Skeleton } from "three";
import { FBXLoader } from "three/examples/jsm/loaders/FBXLoader.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";

const gltfLoader = new GLTFLoader();
gltfLoader.setMeshoptDecoder(MeshoptDecoder);

const fbxLoader = new FBXLoader();

type LoadedAnimationSource = {
  animations: AnimationClip[];
  root: Object3D;
};

export type RuntimeAnimationBundleSource = {
  load: () => Promise<RuntimeAnimationBundle>;
};

export type GameAnimationBundleDefinition = {
  id: string;
  source: RuntimeAnimationBundleSource;
  title?: string;
};

export type LoadedRuntimeAnimationCharacter = {
  rig: RigDefinition;
  root: Object3D;
  skeleton: Skeleton;
  sourceUrl: string;
};

export type RuntimeAnimationBundle = {
  artifact: AnimationArtifact;
  manifest: AnimationBundle;
  rig: RigDefinition | undefined;
  resolveAssetUrl: (path: string) => string;
  loadCharacterAsset: () => Promise<LoadedRuntimeAnimationCharacter | undefined>;
  loadClipAssetsById: (skeleton: Skeleton) => Promise<Record<string, AnimationClipAsset>>;
  loadClipAssetsByName: (skeleton: Skeleton) => Promise<Record<string, AnimationClipAsset>>;
  loadGraphClipAssets: (skeleton: Skeleton) => Promise<AnimationClipAsset[]>;
};

export function defineGameAnimationBundle(definition: GameAnimationBundleDefinition) {
  return definition;
}

export function createPublicRuntimeAnimationSource(manifestUrl: string): RuntimeAnimationBundleSource {
  return {
    async load() {
      const response = await fetch(manifestUrl);

      if (!response.ok) {
        throw new Error(`Failed to load runtime animation bundle from ${manifestUrl}`);
      }

      const manifest = rewriteRuntimeAnimationBundleAssetUrls(
        parseAnimationBundle(await response.json()),
        (path) => absolutizeRuntimeUrl(path, manifestUrl)
      );
      const artifactUrl = absolutizeRuntimeUrl(manifest.artifact, manifestUrl);
      const artifactResponse = await fetch(artifactUrl);

      if (!artifactResponse.ok) {
        throw new Error(`Failed to load animation artifact from ${artifactUrl}`);
      }

      return createRuntimeAnimationBundle({
        artifact: parseAnimationArtifactJson(await artifactResponse.text()),
        manifest,
        resolveAssetUrl: (path) => absolutizeRuntimeUrl(path, manifestUrl)
      });
    }
  };
}

export function createBundledRuntimeAnimationSource(options: {
  artifactText: string;
  assetUrls: Record<string, string>;
  manifestText: string;
}): RuntimeAnimationBundleSource {
  const assetUrls = normalizeBundledAnimationAssetUrls(options.assetUrls);

  return {
    async load() {
      return createRuntimeAnimationBundle({
        artifact: parseAnimationArtifactJson(options.artifactText),
        manifest: rewriteRuntimeAnimationBundleAssetUrls(parseAnimationBundle(JSON.parse(options.manifestText)), (path) => {
          const normalizedPath = normalizeRelativeRuntimePath(path);
          return assetUrls[normalizedPath] ?? path;
        }),
        resolveAssetUrl: (path) => {
          const normalizedPath = normalizeRelativeRuntimePath(path);
          return assetUrls[normalizedPath] ?? path;
        }
      });
    }
  };
}

export function createLazyBundledRuntimeAnimationSource(options: {
  assetUrls: Record<string, string>;
  loadArtifactText: () => Promise<string>;
  loadManifestText: () => Promise<string>;
}): RuntimeAnimationBundleSource {
  const assetUrls = normalizeBundledAnimationAssetUrls(options.assetUrls);

  return {
    async load() {
      const [artifactText, manifestText] = await Promise.all([
        options.loadArtifactText(),
        options.loadManifestText()
      ]);

      return createRuntimeAnimationBundle({
        artifact: parseAnimationArtifactJson(artifactText),
        manifest: rewriteRuntimeAnimationBundleAssetUrls(parseAnimationBundle(JSON.parse(manifestText)), (path) => {
          const normalizedPath = normalizeRelativeRuntimePath(path);
          return assetUrls[normalizedPath] ?? path;
        }),
        resolveAssetUrl: (path) => {
          const normalizedPath = normalizeRelativeRuntimePath(path);
          return assetUrls[normalizedPath] ?? path;
        }
      });
    }
  };
}

export function normalizeBundledAnimationAssetUrls(assetUrls: Record<string, string>) {
  return Object.fromEntries(
    Object.entries(assetUrls).map(([key, value]) => [normalizeRelativeRuntimePath(key), value])
  );
}

function createRuntimeAnimationBundle(input: {
  artifact: AnimationArtifact;
  manifest: AnimationBundle;
  resolveAssetUrl: (path: string) => string;
}): RuntimeAnimationBundle {
  const rig = loadRigFromArtifact(input.artifact);
  const artifactClipsById = new Map(loadClipsFromArtifact(input.artifact).map((clip) => [clip.id, clip]));
  const animationSourceCache = new Map<string, Promise<LoadedAnimationSource>>();

  const loadAnimationSourceCached = (assetUrl: string) => {
    let pending = animationSourceCache.get(assetUrl);
    if (!pending) {
      pending = loadAnimationSource(assetUrl);
      animationSourceCache.set(assetUrl, pending);
    }
    return pending;
  };

  return {
    artifact: input.artifact,
    manifest: input.manifest,
    rig,
    resolveAssetUrl(path: string) {
      return resolveRuntimeAssetPath(path, input.resolveAssetUrl);
    },
    async loadCharacterAsset() {
      if (!input.manifest.characterAsset) {
        return undefined;
      }

      const sourceUrl = resolveRuntimeAssetPath(input.manifest.characterAsset, input.resolveAssetUrl);
      const source = await loadAnimationSourceCached(sourceUrl);
      const skeleton = findPrimarySkeleton(source.root);

      if (!skeleton) {
        throw new Error(`Animation character asset ${sourceUrl} does not contain a skinned skeleton.`);
      }

      return {
        rig: createRigFromSkeleton(skeleton),
        root: source.root,
        skeleton,
        sourceUrl
      };
    },
    async loadClipAssetsById(skeleton: Skeleton) {
      const entries = await Promise.all(
        input.manifest.clips.map(async (clipEntry) => {
          const artifactClip = artifactClipsById.get(clipEntry.id);

          if (artifactClip) {
            return [clipEntry.id, artifactClip] as const;
          }

          if (!clipEntry.asset) {
            throw new Error(`Animation bundle clip ${clipEntry.id} is missing both embedded artifact data and an external asset path.`);
          }

          const assetUrl = resolveRuntimeAssetPath(clipEntry.asset, input.resolveAssetUrl);
          const source = await loadAnimationSourceCached(assetUrl);
          const clip = resolveAnimationClipFromSource(source.animations, clipEntry.name, clipEntry.id, assetUrl);
          const asset = createClipAssetFromThreeClip(clip, skeleton);

          return [
            clipEntry.id,
            {
              ...asset,
              duration: clipEntry.duration,
              id: clipEntry.id,
              name: clipEntry.name
            }
          ] as const;
        })
      );

      return Object.fromEntries(entries);
    },
    async loadClipAssetsByName(skeleton: Skeleton) {
      const clipsById = await this.loadClipAssetsById(skeleton);

      return Object.fromEntries(input.manifest.clips.map((clipEntry) => [clipEntry.name, clipsById[clipEntry.id]]));
    },
    async loadGraphClipAssets(skeleton: Skeleton) {
      const clipsById = await this.loadClipAssetsById(skeleton);

      return input.artifact.graph.clipSlots.map((clipSlot) => {
        const clip = clipsById[clipSlot.id];

        if (!clip) {
          throw new Error(`Animation bundle is missing clip asset data for slot ${clipSlot.id}.`);
        }

        return clip;
      });
    }
  };
}

function rewriteRuntimeAnimationBundleAssetUrls(
  bundle: AnimationBundle,
  resolveAssetUrl: (path: string) => string
): AnimationBundle {
  const rewritten = structuredClone(bundle);

  rewritten.clips = rewritten.clips.map((clip) => ({
    ...clip,
    asset: clip.asset ? resolveRuntimeAssetPath(clip.asset, resolveAssetUrl) : undefined
  }));
  rewritten.clipAssets = Object.fromEntries(
    Object.entries(rewritten.clipAssets).map(([clipName, assetPath]) => [
      clipName,
      resolveRuntimeAssetPath(assetPath, resolveAssetUrl)
    ])
  );

  if (rewritten.characterAsset) {
    rewritten.characterAsset = resolveRuntimeAssetPath(rewritten.characterAsset, resolveAssetUrl);
  }

  return rewritten;
}

function resolveRuntimeAssetPath(path: string, resolveAssetUrl: (path: string) => string) {
  if (!path || isAbsoluteRuntimeUrl(path)) {
    return path;
  }

  return resolveAssetUrl(path);
}

function absolutizeRuntimeUrl(path: string, manifestUrl: string) {
  if (isAbsoluteRuntimeUrl(path)) {
    return path;
  }

  return new URL(path, new URL(manifestUrl, window.location.origin)).toString();
}

function isAbsoluteRuntimeUrl(path: string) {
  return /^[a-z]+:/i.test(path) || path.startsWith("//") || path.startsWith("/");
}

function normalizeRelativeRuntimePath(path: string) {
  return path.replace(/^\.\//, "");
}

function getFileExtensionFromUrl(url: string) {
  const withoutHash = url.split("#", 1)[0] ?? url;
  const withoutQuery = withoutHash.split("?", 1)[0] ?? withoutHash;
  return withoutQuery.split(".").pop()?.toLowerCase() ?? "";
}

async function loadAnimationSource(url: string): Promise<LoadedAnimationSource> {
  const extension = getFileExtensionFromUrl(url);

  if (extension === "glb" || extension === "gltf") {
    const result = await gltfLoader.loadAsync(url);
    return {
      animations: result.animations,
      root: result.scene
    };
  }

  if (extension === "fbx") {
    const result = await fbxLoader.loadAsync(url);
    return {
      animations: result.animations,
      root: result
    };
  }

  throw new Error(`Unsupported runtime animation asset type .${extension || "unknown"}.`);
}

function findPrimarySkeleton(root: Object3D): Skeleton | null {
  let foundSkeleton: Skeleton | null = null;

  root.traverse((child) => {
    if (foundSkeleton) {
      return;
    }

    const candidate = child as Object3D & {
      isSkinnedMesh?: boolean;
      skeleton?: Skeleton;
    };

    if (candidate.isSkinnedMesh && candidate.skeleton) {
      foundSkeleton = candidate.skeleton;
    }
  });

  return foundSkeleton;
}

function resolveAnimationClipFromSource(
  animations: AnimationClip[],
  clipName: string,
  clipId: string,
  assetUrl: string
) {
  const matchedByName = animations.find((animation) => animation.name === clipName);

  if (matchedByName) {
    return matchedByName;
  }

  const matchedById = animations.find((animation) => animation.name === clipId);

  if (matchedById) {
    return matchedById;
  }

  if (animations.length === 1 && animations[0]) {
    return animations[0];
  }

  throw new Error(`Animation asset ${assetUrl} does not contain a clip named ${clipName}.`);
}
