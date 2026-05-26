import { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";
import { ReactElement, useCallback, useState } from "react";

import { Snapshot } from "../interfaces";

// Save-plot-to-disk buttons. A Foxglove panel can't write files itself, so
// each button calls a ROS service (std_srvs/Trigger) on the snapshot
// driver, which renders the plot to a PNG and saves it to the laptop
// Desktop (fallback: <repo>/snapshots/). The service returns the saved
// path, which we show here.

interface TriggerResponse {
  success?: boolean;
  message?: string;
}

interface Result {
  ok: boolean;
  text: string;
}

function SnapshotsPanel(props: { context: PanelExtensionContext }): ReactElement {
  const { context } = props;
  const [busy, setBusy] = useState<string | undefined>();
  const [result, setResult] = useState<Result | undefined>();

  const haveCallService = typeof context.callService === "function";

  const call = useCallback(
    (service: string, label: string) => async () => {
      if (busy !== undefined) return;
      setBusy(label);
      setResult(undefined);
      try {
        const resp = (await context.callService?.(service, {})) as
          | TriggerResponse
          | undefined;
        if (resp?.success) {
          setResult({ ok: true, text: `Saved: ${resp.message ?? "(no path)"}` });
        } else {
          setResult({ ok: false, text: resp?.message ?? "(no response)" });
        }
      } catch (err) {
        setResult({ ok: false, text: String(err) });
      } finally {
        setBusy(undefined);
      }
    },
    [context, busy],
  );

  const disabled = !haveCallService || busy !== undefined;

  return (
    <div
      style={{
        padding: "12px 16px",
        fontFamily: "sans-serif",
        fontSize: 13,
        color: "var(--foxglove-text, #e8e8e8)",
        height: "100%",
        boxSizing: "border-box",
        overflow: "auto",
      }}
    >
      <h3 style={{ marginTop: 0, marginBottom: 4 }}>Snapshots</h3>
      <div style={{ opacity: 0.7, marginBottom: 12, fontSize: 12 }}>
        Saves the current plot as a PNG to the laptop Desktop
        (<code>~/Desktop/astrotech_snapshots/</code>), or the project
        <code> snapshots/</code> folder if the Desktop isn&apos;t writable.
      </div>

      {!haveCallService && (
        <div
          style={{
            background: "rgba(255,100,100,0.15)",
            border: "1px solid rgba(255,100,100,0.4)",
            borderRadius: 4,
            padding: 8,
            marginBottom: 12,
          }}
        >
          This data source can&apos;t call services. Connect through the
          Foxglove WebSocket bridge to use the snapshot buttons.
        </div>
      )}

      <div style={{ display: "flex", gap: 8 }}>
        <SaveButton
          label="Save Raman"
          active={busy === "Raman"}
          disabled={disabled}
          onClick={call(Snapshot.saveRamanService, "Raman")}
        />
        <SaveButton
          label="Save CO₂"
          active={busy === "CO₂"}
          disabled={disabled}
          onClick={call(Snapshot.saveCo2Service, "CO₂")}
        />
      </div>

      {result && (
        <div
          style={{
            marginTop: 12,
            fontSize: 12,
            lineHeight: 1.5,
            color: result.ok ? "#7fcf7f" : "#e57373",
            wordBreak: "break-all",
          }}
        >
          {result.text}
        </div>
      )}
    </div>
  );
}

function SaveButton(props: {
  label: string;
  active?: boolean;
  disabled?: boolean;
  onClick: () => void;
}): ReactElement {
  return (
    <button
      type="button"
      disabled={props.disabled}
      onClick={() => {
        if (!props.disabled) props.onClick();
      }}
      style={{
        flex: 1,
        padding: "10px 12px",
        background: props.active
          ? "var(--foxglove-accent, #5469d4)"
          : "rgba(255,255,255,0.08)",
        color: props.disabled ? "rgba(255,255,255,0.3)" : "var(--foxglove-text, #e8e8e8)",
        border: "1px solid rgba(255,255,255,0.15)",
        borderRadius: 4,
        cursor: props.disabled ? "not-allowed" : "pointer",
        fontSize: 13,
        fontWeight: 600,
      }}
    >
      {props.label}
      {props.active ? " …" : ""}
    </button>
  );
}

export function initSnapshotsPanel(
  context: PanelExtensionContext,
): () => void {
  const root = createRoot(context.panelElement);
  root.render(<SnapshotsPanel context={context} />);
  return () => {
    root.unmount();
  };
}
