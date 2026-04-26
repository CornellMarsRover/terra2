import { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";

import { Analysis } from "../interfaces";
import { PanelStub } from "./PanelStub";

export function initAnalysisSequencePanel(
  context: PanelExtensionContext,
): () => void {
  const root = createRoot(context.panelElement);

  // Phase 2a: stub buttons log only. Phase 2b will open the action clients,
  // disable buttons while a sequence is in flight, and render feedback
  // (progress_pct + current_step) live.
  //
  // TODO(astrotech-q-3), TODO(astrotech-q-4): labels here assume "Seq N"
  // semantics. If the Astrotech team responds with site-based naming the
  // button labels change; the topic strings already come from interfaces.ts.
  const buttons = Analysis.sequences.map((seq) => ({
    label: `Start Seq ${seq.id}`,
    serviceName: seq.action,
    payloadSummary: { sequence_id: seq.id },
  }));

  root.render(
    <PanelStub
      context={context}
      spec={{
        title: "Analysis sequence",
        buttons,
      }}
    />
  );

  return () => {
    root.unmount();
  };
}
