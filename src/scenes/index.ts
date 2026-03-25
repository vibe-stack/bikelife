import { arenaScene } from "./arena";
import { mainScene } from "./main";
import { rampTest } from "./ramp-test";

export const scenes = {
  [arenaScene.id]: arenaScene,
  [mainScene.id]: mainScene,
  [rampTest.id]: rampTest
};

export const initialSceneId = "ramp-test";
