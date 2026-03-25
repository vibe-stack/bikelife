import {
  createLazyBundledRuntimeAnimationSource,
  defineGameAnimationBundle
} from "../../game/runtime-animation-sources";

const assetUrls = import.meta.glob("./assets/**/*", {
  eager: true,
  import: "default",
  query: "?url"
}) as Record<string, string>;

export const playerAnimations = defineGameAnimationBundle({
  id: "player",
  source: createLazyBundledRuntimeAnimationSource({
    assetUrls,
    loadArtifactText: () => import("./graph.animation.json?raw").then((module) => module.default),
    loadManifestText: () => import("./animation.bundle.json?raw").then((module) => module.default)
  }),
  title: "Player"
});
