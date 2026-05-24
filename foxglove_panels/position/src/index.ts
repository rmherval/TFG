import { ExtensionContext } from "@foxglove/extension";

import { initRobotPosePanel } from "./RobotPosePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "position-panel", initPanel: initRobotPosePanel });
}
