import { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";

import { MixingServo } from "../interfaces";
import { PanelStub } from "./PanelStub";

export function initMixingServoPanel(
  context: PanelExtensionContext,
): () => void {
  const root = createRoot(context.panelElement);

  // Phase 2a: one button per preset; click logs the service name + preset.
  // Phase 2b will call the service for real, disable the panel while a
  // previous call is in flight, and show current position from stateTopic.
  const buttons = MixingServo.presets.map((preset) => ({
    label: preset,
    serviceName: MixingServo.setPresetService,
    payloadSummary: { preset_name: preset },
  }));

  root.render(
    <PanelStub
      context={context}
      spec={{
        title: "Mixing servo",
        subscriptions: [
          { topic: MixingServo.stateTopic, label: "current preset" },
        ],
        buttons,
      }}
    />
  );

  return () => {
    root.unmount();
  };
}
