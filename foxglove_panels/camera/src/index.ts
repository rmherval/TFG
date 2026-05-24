import { ExtensionContext } from "@foxglove/extension";

import { initCameraPanel } from "./CameraPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "camera-panel", initPanel: initCameraPanel });
}
