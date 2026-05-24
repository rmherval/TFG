import { ExtensionContext } from "@foxglove/extension";

import { initBatteryPanel } from "./BatteryPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "battery-panel", initPanel: initBatteryPanel });
}
