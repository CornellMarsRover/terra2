import { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";
import {
  ReactElement,
  useCallback,
  useEffect,
  useLayoutEffect,
  useMemo,
  useState,
} from "react";

import { Analysis } from "../interfaces";

// Analysis sequence panel. Foxglove panels have no ROS action client, so the
// rover exposes a Foxglove-native interface (see interfaces.ts / the rover's
// analysis_sequencer.py): a start service per sequence id, a cancel Trigger,
// and an AnalysisStatus topic we subscribe to for live progress. The boards
// give no feedback, so progress is commanded-only and Cancel is a hard abort.

interface AnalysisStatusMessage {
  running: boolean;
  active_sequence_id: number;
  current_step: number;
  total_steps: number;
  current_step_description: string;
  elapsed_seconds: number;
  last_success: boolean;
  last_message: string;
  last_completed_step: number;
}

interface ServiceResponse {
  success?: boolean;
  message?: string;
}

interface CallResult {
  ok: boolean;
  msg: string;
  label: string;
}

function AnalysisPanel(props: { context: PanelExtensionContext }): ReactElement {
  const { context } = props;

  const [status, setStatus] = useState<AnalysisStatusMessage | undefined>();
  const [busyLabel, setBusyLabel] = useState<string | undefined>();
  const [lastResult, setLastResult] = useState<CallResult | undefined>();
  const [renderDone, setRenderDone] = useState<() => void>();

  const subscriptions = useMemo(() => [{ topic: Analysis.statusTopic }], []);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frames = renderState.currentFrame ?? [];
      for (const frame of frames) {
        if (frame.topic === Analysis.statusTopic) {
          setStatus(frame.message as AnalysisStatusMessage);
        }
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

  const haveCallService = typeof context.callService === "function";
  const running = status?.running ?? false;

  const startSeq = useCallback(
    (id: number, label: string) => async () => {
      if (busyLabel !== undefined) return;
      setBusyLabel(label);
      setLastResult(undefined);
      try {
        const resp = (await context.callService?.(Analysis.startService, {
          sequence_id: id,
        })) as ServiceResponse | undefined;
        setLastResult({
          ok: !!resp?.success,
          msg: resp?.message ?? "(no response)",
          label,
        });
      } catch (err) {
        setLastResult({ ok: false, msg: String(err), label });
      } finally {
        setBusyLabel(undefined);
      }
    },
    [context, busyLabel],
  );

  const cancel = useCallback(async () => {
    if (busyLabel !== undefined) return;
    setBusyLabel("Cancel");
    setLastResult(undefined);
    try {
      const resp = (await context.callService?.(Analysis.cancelService, {})) as
        | ServiceResponse
        | undefined;
      setLastResult({
        ok: !!resp?.success,
        msg: resp?.message ?? "(no response)",
        label: "Cancel",
      });
    } catch (err) {
      setLastResult({ ok: false, msg: String(err), label: "Cancel" });
    } finally {
      setBusyLabel(undefined);
    }
  }, [context, busyLabel]);

  const startDisabled = !haveCallService || busyLabel !== undefined || running;

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
      <h3 style={{ marginTop: 0, marginBottom: 4 }}>Analysis sequence</h3>
      <div style={{ opacity: 0.7, marginBottom: 12, fontSize: 12 }}>
        Start a science assay; the rover runs it step by step and reports
        progress here. Cancel is a hard abort (stops every pump). Only one
        sequence runs at a time.
      </div>

      {!haveCallService && (
        <div
          style={{
            background: "rgba(255, 100, 100, 0.15)",
            border: "1px solid rgba(255,100,100,0.4)",
            borderRadius: 4,
            padding: 8,
            marginBottom: 12,
          }}
        >
          The current data source does not support service calls. Connect via
          the Foxglove WebSocket bridge to use this panel.
        </div>
      )}

      <Group label="Start">
        {Analysis.sequences.map((seq) => (
          <ActionButton
            key={seq.id}
            label={`Start ${seq.name}`}
            subtitle={`sequence ${seq.id}`}
            active={busyLabel === seq.name}
            disabled={startDisabled}
            onClick={startSeq(seq.id, seq.name)}
          />
        ))}
      </Group>

      <Group label="Abort">
        <CancelButton
          disabled={!haveCallService || busyLabel !== undefined || !running}
          onClick={cancel}
        />
      </Group>

      <Progress status={status} busyLabel={busyLabel} lastResult={lastResult} />
    </div>
  );
}

function Group(props: {
  label: string;
  children: ReactElement | ReactElement[];
}): ReactElement {
  return (
    <div style={{ marginBottom: 12 }}>
      <div
        style={{
          fontSize: 11,
          opacity: 0.6,
          textTransform: "uppercase",
          letterSpacing: 0.5,
          marginBottom: 4,
        }}
      >
        {props.label}
      </div>
      <div style={{ display: "grid", gridTemplateColumns: "1fr", gap: 8 }}>
        {props.children}
      </div>
    </div>
  );
}

function ActionButton(props: {
  label: string;
  subtitle?: string;
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
        padding: "10px 12px",
        background: props.active
          ? "var(--foxglove-accent, #5469d4)"
          : "rgba(255,255,255,0.08)",
        color: props.disabled
          ? "rgba(255,255,255,0.3)"
          : props.active
            ? "#fff"
            : "var(--foxglove-text, #e8e8e8)",
        border: "1px solid rgba(255,255,255,0.15)",
        borderRadius: 4,
        cursor: props.disabled ? "not-allowed" : "pointer",
        userSelect: "none",
        fontSize: 13,
        textAlign: "left",
        lineHeight: 1.3,
        opacity: props.disabled && !props.active ? 0.55 : 1,
      }}
    >
      <div style={{ fontWeight: 600 }}>
        {props.label}
        {props.active ? " …" : ""}
      </div>
      {props.subtitle && (
        <div style={{ fontSize: 11, opacity: 0.65, marginTop: 2 }}>
          {props.subtitle}
        </div>
      )}
    </button>
  );
}

function CancelButton(props: {
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
        padding: "12px 12px",
        background: props.disabled ? "rgba(155,56,56,0.4)" : "#9b3838",
        color: props.disabled ? "rgba(255,255,255,0.4)" : "#fff",
        border: "1px solid #c25151",
        borderRadius: 4,
        cursor: props.disabled ? "not-allowed" : "pointer",
        userSelect: "none",
        fontSize: 14,
        fontWeight: 700,
        textAlign: "center",
        textTransform: "uppercase",
        letterSpacing: 0.5,
      }}
    >
      Cancel
    </button>
  );
}

function Progress(props: {
  status: AnalysisStatusMessage | undefined;
  busyLabel: string | undefined;
  lastResult: CallResult | undefined;
}): ReactElement {
  const s = props.status;
  const running = s?.running ?? false;
  const total = s?.total_steps ?? 0;
  const step = s?.current_step ?? 0;
  const pct = running && total > 0 ? Math.min(100, (step / total) * 100) : 0;

  return (
    <div
      style={{
        borderTop: "1px solid rgba(255,255,255,0.12)",
        paddingTop: 8,
        fontSize: 12,
        lineHeight: 1.6,
      }}
    >
      <div style={{ fontWeight: 600, marginBottom: 4 }}>Progress</div>

      {!s && (
        <div style={{ opacity: 0.6 }}>no {Analysis.statusTopic} yet</div>
      )}

      {s && running && (
        <>
          <div>
            running sequence <code>{s.active_sequence_id}</code> — step{" "}
            <code>
              {step}/{total}
            </code>{" "}
            · {s.elapsed_seconds.toFixed(1)} s
          </div>
          <div style={{ opacity: 0.8, margin: "2px 0 6px" }}>
            {s.current_step_description}
          </div>
          <div
            style={{
              height: 8,
              background: "rgba(255,255,255,0.1)",
              borderRadius: 4,
              overflow: "hidden",
            }}
          >
            <div
              style={{
                width: `${pct}%`,
                height: "100%",
                background: "var(--foxglove-accent, #5469d4)",
                transition: "width 0.2s",
              }}
            />
          </div>
        </>
      )}

      {s && !running && (
        <div>
          status: <span style={{ opacity: 0.7 }}>idle</span>
          {s.last_message ? (
            <div
              style={{
                marginTop: 4,
                color: s.last_success ? "#7fcf7f" : "#e57373",
              }}
            >
              last: {s.last_success ? "OK" : "ended"} — {s.last_message}
            </div>
          ) : null}
        </div>
      )}

      {props.lastResult && !props.lastResult.ok && (
        <div style={{ marginTop: 6, color: "#e57373" }}>
          {props.lastResult.label}: {props.lastResult.msg}
        </div>
      )}
    </div>
  );
}

export function initAnalysisSequencePanel(
  context: PanelExtensionContext,
): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AnalysisPanel context={context} />);
  return () => {
    root.unmount();
  };
}
