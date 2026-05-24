import { ExtensionContext } from "@foxglove/extension";

import { initENavOutdoorPanel } from "./ENavOutdoor";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "easynav-outdoor", initPanel: initENavOutdoorPanel });
}
