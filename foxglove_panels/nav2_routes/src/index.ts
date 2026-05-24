import { ExtensionContext } from "@foxglove/extension";

import { initNav2OutdoorPanel } from "./Nav2Outdoor";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "nav2-routes", initPanel: initNav2OutdoorPanel });
}

