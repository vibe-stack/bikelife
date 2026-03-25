import { defineConfig, searchForWorkspaceRoot } from "vite";
import { createWebHammerGamePlugin } from "@ggez/game-dev";

export default defineConfig({
  plugins: [createWebHammerGamePlugin({ initialSceneId: "ramp-test", projectName: "meek" })],
  server: {
    fs: {
      allow: [searchForWorkspaceRoot(process.cwd())]
    }
  }
});
