import type { ExtensionContext } from "@foxglove/extension";

import { initOperatorPanel } from "./OperatorPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Sahabat Operator",
    initPanel: initOperatorPanel,
  });
}
