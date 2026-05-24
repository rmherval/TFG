import { ExtensionContext } from "@foxglove/extension";

import { initVisualization3DPanel } from "./Visualization3DPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "visualization-3d", initPanel: initVisualization3DPanel });
}
