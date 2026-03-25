import {
  createLazyBundledRuntimeSceneSource,
  defineGameScene,
  normalizeBundledAssetUrls
} from "../../game/runtime-scene-sources";

const assetUrls = normalizeBundledAssetUrls(
  import.meta.glob("./assets/**/*", {
    eager: true,
    import: "default",
    query: "?url"
  }) as Record<string, string>
);

export const arenaScene = defineGameScene({
  id: "arena",
  mount({ gotoScene, player, setStatus }) {
    if (player) {
      setStatus("Arena scene. Click to recapture the cursor if needed. Press 1 to return to Main.");
    }

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.code === "Digit1") {
        void gotoScene("main");
      }
    };

    window.addEventListener("keydown", handleKeyDown);

    return {
      dispose() {
        window.removeEventListener("keydown", handleKeyDown);
      }
    };
  },
  source: createLazyBundledRuntimeSceneSource({
    assetUrls,
    loadManifestText: () => import("./scene.runtime.json?raw").then((module) => module.default)
  }),
  title: "Arena Scene"
});
