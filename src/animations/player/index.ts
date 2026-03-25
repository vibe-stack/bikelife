import animationBundleUrl from "./animation.bundle.json?url";
import {
  createPublicRuntimeAnimationSource,
  defineGameAnimationBundle
} from "../../game/runtime-animation-sources";

export const playerAnimations = defineGameAnimationBundle({
  id: "player",
  source: createPublicRuntimeAnimationSource(animationBundleUrl),
  title: "Player"
});
