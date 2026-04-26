import { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";
import {
  ReactElement,
  useEffect,
  useLayoutEffect,
  useMemo,
  useState,
} from "react";

import { Raman } from "../interfaces";

// Phase 2a: show "last spectrum: N points received at T". No plot yet.
// Phase 2b replaces this with a wavelength-on-X / intensity-on-Y chart;
// Foxglove's built-in Plot is time-series-only, so a custom panel is the
// only option for spectrum rendering.
function RamanPanel(props: { context: PanelExtensionContext }): ReactElement {
  const { context } = props;
  const [summary, setSummary] = useState<string>("no spectrum yet");
  const [renderDone, setRenderDone] = useState<() => void>();

  const subscriptions = useMemo(
    () => [{ topic: Raman.topic }],
    [],
  );

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frames = renderState.currentFrame ?? [];
      for (const frame of frames) {
        if (frame.topic !== Raman.topic) {
          continue;
        }
        // `message` type is opaque at compile time; introspect defensively.
        const msg = frame.message as {
          intensities?: number[] | Float32Array;
          wavenumbers_cm_inv?: number[] | Float32Array;
        };
        const n = msg.intensities?.length ?? 0;
        setSummary(
          `last spectrum: ${n} points received at ${new Date().toISOString()}`,
        );
      }
    };
    context.watch("currentFrame");
  }, [context]);

  useEffect(() => {
    context.subscribe(subscriptions);
  }, [context, subscriptions]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div
      style={{
        padding: "12px 16px",
        fontFamily: "sans-serif",
        fontSize: 13,
        color: "var(--foxglove-text, #e8e8e8)",
        height: "100%",
        boxSizing: "border-box",
      }}
    >
      <h3 style={{ marginTop: 0, marginBottom: 8 }}>Raman spectrum</h3>
      <div style={{ opacity: 0.8, marginBottom: 12 }}>
        Phase 2a stub — real wavelength-vs-intensity plot lands in Phase 2b.
      </div>
      <div>
        <code>{Raman.topic}</code>
      </div>
      <div style={{ marginTop: 8 }}>{summary}</div>
    </div>
  );
}

export function initRamanSpectrumPanel(
  context: PanelExtensionContext,
): () => void {
  const root = createRoot(context.panelElement);
  root.render(<RamanPanel context={context} />);
  return () => {
    root.unmount();
  };
}
