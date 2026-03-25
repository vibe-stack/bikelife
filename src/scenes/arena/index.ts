import sceneManifestUrl from "./scene.runtime.json?url";
import { createPublicRuntimeSceneSource, defineGameScene } from "../../game/runtime-scene-sources";

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
  source: createPublicRuntimeSceneSource(sceneManifestUrl),
  title: "Arena Scene"
});
