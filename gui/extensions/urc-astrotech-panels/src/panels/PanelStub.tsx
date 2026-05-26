import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useState } from "react";

/** Minimal subscription entry the stubs report back on-screen. */
export interface SubscriptionSpec {
  /** Topic name to subscribe to. */
  topic: string;
  /** Optional human-readable label shown in the "subscribed to:" list. */
  label?: string;
}

/** Service buttons for the stub to render. Phase 2a logs only; no real call. */
export interface ServiceButton {
  label: string;
  /** Service name that would be called if this were Phase 2b. */
  serviceName: string;
  /** Request payload shape (printed to console). */
  payloadSummary?: Record<string, unknown>;
}

export interface PanelStubProps {
  title: string;
  subscriptions?: SubscriptionSpec[];
  buttons?: ServiceButton[];
  /** Extra content rendered beneath the status block (e.g. spectrum stats). */
  extra?: ReactElement | null;
}

/**
 * Shared UI for all Phase 2a panel stubs.
 *
 * Renders:
 *  - Panel title.
 *  - Subscribed-topics list and most-recent-message timestamp (one line per
 *    topic).
 *  - Buttons that, when clicked, log the service + payload to the browser
 *    console. No service calls are issued in Phase 2a.
 *
 * Phase 2b will replace per-panel files with real UIs that keep the same
 * subscriptions / service names.
 */
export function PanelStub(props: {
  context: PanelExtensionContext;
  spec: PanelStubProps;
}): JSX.Element {
  const { context, spec } = props;
  const subs = useMemo(() => spec.subscriptions ?? [], [spec.subscriptions]);
  const buttons = spec.buttons ?? [];

  const [lastMessageByTopic, setLastMessageByTopic] = useState<
    Record<string, string>
  >({});
  const [renderDone, setRenderDone] = useState<() => void>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frames = renderState.currentFrame ?? [];
      if (frames.length === 0) {
        return;
      }
      const nowIso = new Date().toISOString();
      setLastMessageByTopic((prev) => {
        const next = { ...prev };
        for (const frame of frames) {
          next[frame.topic] = nowIso;
        }
        return next;
      });
    };
    context.watch("currentFrame");
  }, [context]);

  useEffect(() => {
    if (subs.length > 0) {
      context.subscribe(subs.map((s) => ({ topic: s.topic })));
    }
  }, [context, subs]);

  // Foxglove requires the panel to call the render-done callback each frame.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const handleClick = (btn: ServiceButton) => () => {
    // Phase 2a is logging-only — a sane default so Phase 2b has something
    // obvious to replace.
    // eslint-disable-next-line no-console
    console.log(
      `[${spec.title}] Phase 2a stub: would call`,
      btn.serviceName,
      "with",
      btn.payloadSummary ?? {}
    );
  };

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
      <h3 style={{ marginTop: 0, marginBottom: 8 }}>{spec.title}</h3>
      <div style={{ opacity: 0.8, marginBottom: 12 }}>
        Phase 2a stub — logic lands in Phase 2b.
      </div>

      {subs.length > 0 && (
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontWeight: 600 }}>Subscribed to:</div>
          <ul style={{ margin: "4px 0 0 0", paddingLeft: 18 }}>
            {subs.map((s) => (
              <li key={s.topic}>
                <code>{s.topic}</code>
                {s.label ? ` — ${s.label}` : ""}
                <span style={{ opacity: 0.6, marginLeft: 8 }}>
                  last message:{" "}
                  {lastMessageByTopic[s.topic] ?? "—"}
                </span>
              </li>
            ))}
          </ul>
        </div>
      )}

      {buttons.length > 0 && (
        <div style={{ display: "flex", flexWrap: "wrap", gap: 8 }}>
          {buttons.map((btn) => (
            <button
              key={btn.label + btn.serviceName}
              type="button"
              onClick={handleClick(btn)}
              style={{
                padding: "6px 12px",
                background: "var(--foxglove-accent, #5469d4)",
                color: "#fff",
                border: "none",
                borderRadius: 4,
                cursor: "pointer",
                fontSize: 13,
              }}
            >
              {btn.label}
            </button>
          ))}
        </div>
      )}

      {spec.extra}
    </div>
  );
}
