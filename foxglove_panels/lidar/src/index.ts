import { ExtensionContext } from "@foxglove/extension";

import { initLidarPanel } from "./LidarPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "LidarPanel", initPanel: initLidarPanel});
}
