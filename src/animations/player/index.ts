import finalOptUrl from "./assets/final-opt.glb?url";
import graphAnimationText from "./graph.animation.json?raw";
import animationBundleText from "./animation.bundle.json?raw";
import {
  createBundledRuntimeAnimationSource,
  defineGameAnimationBundle
} from "../../game/runtime-animation-sources";

export const playerAnimations = defineGameAnimationBundle({
  id: "player",
  source: createBundledRuntimeAnimationSource({
    artifactText: graphAnimationText,
    assetUrls: {
      "assets/final-opt.glb": finalOptUrl
    },
    manifestText: animationBundleText
  }),
  title: "Player"
});
