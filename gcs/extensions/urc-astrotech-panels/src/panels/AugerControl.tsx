import { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";

import { Auger } from "../interfaces";
import { PanelStub } from "./PanelStub";

export function initAugerControlPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);

  // Phase 2a: stub buttons only. Phase 2b will replace these with a real
  // velocity slider + momentary Up/Down/Spin controls that publish to
  // Auger.cmdTopic.
  //
  // TODO(astrotech-q-1): if Auger.cmdType changes away from Twist the
  // payloadSummary entries below must change to match the new message.
  root.render(
    <PanelStub
      context={context}
      spec={{
        title: "Auger control",
        subscriptions: [
          { topic: Auger.stateTopic, label: "auger state" },
        ],
        buttons: [
          {
            label: "Up",
            serviceName: Auger.cmdTopic,
            payloadSummary: { "linear.z": 1.0, "angular.z": 0.0 },
          },
          {
            label: "Down",
            serviceName: Auger.cmdTopic,
            payloadSummary: { "linear.z": -1.0, "angular.z": 0.0 },
          },
          {
            label: "Spin Fwd",
            serviceName: Auger.cmdTopic,
            payloadSummary: { "linear.z": 0.0, "angular.z": 1.0 },
          },
          {
            label: "Spin Back",
            serviceName: Auger.cmdTopic,
            payloadSummary: { "linear.z": 0.0, "angular.z": -1.0 },
          },
          {
            label: "Stop",
            serviceName: Auger.cmdTopic,
            payloadSummary: { "linear.z": 0.0, "angular.z": 0.0 },
          },
        ],
      }}
    />
  );

  return () => {
    root.unmount();
  };
}
