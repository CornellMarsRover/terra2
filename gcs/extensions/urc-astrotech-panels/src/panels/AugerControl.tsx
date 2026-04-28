import { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";
import {
  ReactElement,
  useCallback,
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
} from "react";

import { Auger } from "../interfaces";

// Hold-to-act direction flags. The panel tracks which buttons are currently
// pressed (mousedown/touchstart -> true, mouseup/touchend/leave -> false) and
// publishes an AugerCommand at PUBLISH_HZ summing the active directions.
type Held = {
  up: boolean;
  down: boolean;
  spinFwd: boolean;
  spinBack: boolean;
};

const ZERO_HELD: Held = { up: false, down: false, spinFwd: false, spinBack: false };

interface AugerCommandMessage {
  lead_screw_velocity_rev_s: number;
  lead_screw_max_torque_nm: number;
  auger_velocity_rev_s: number;
  auger_max_torque_nm: number;
}

const ZERO_CMD: AugerCommandMessage = {
  lead_screw_velocity_rev_s: 0.0,
  lead_screw_max_torque_nm: Auger.defaults.leadScrewMaxTorqueNm,
  auger_velocity_rev_s: 0.0,
  auger_max_torque_nm: Auger.defaults.augerMaxTorqueNm,
};

function commandFromHeld(held: Held): AugerCommandMessage {
  let leadVel = 0.0;
  if (held.up) leadVel += Auger.defaults.leadScrewVelRevS;
  if (held.down) leadVel -= Auger.defaults.leadScrewVelRevS;

  let augerVel = 0.0;
  if (held.spinFwd) augerVel += Auger.defaults.augerSpinForwardRevS;
  if (held.spinBack) augerVel -= Auger.defaults.augerSpinBackwardRevS;

  return {
    lead_screw_velocity_rev_s: leadVel,
    lead_screw_max_torque_nm: Auger.defaults.leadScrewMaxTorqueNm,
    auger_velocity_rev_s: augerVel,
    auger_max_torque_nm: Auger.defaults.augerMaxTorqueNm,
  };
}

interface AugerStateMessage {
  lead_screw_position_rev: number;
  lead_screw_velocity_rev_s: number;
  lead_screw_torque_nm: number;
  lead_screw_temperature_c: number;
  lead_screw_mode: number;
  lead_screw_fault: number;
  auger_position_rev: number;
  auger_velocity_rev_s: number;
  auger_torque_nm: number;
  auger_temperature_c: number;
  auger_mode: number;
  auger_fault: number;
}

function fmt(n: number | undefined, digits = 2): string {
  return typeof n === "number" && Number.isFinite(n) ? n.toFixed(digits) : "—";
}

function AugerPanel(props: { context: PanelExtensionContext }): ReactElement {
  const { context } = props;
  const heldRef = useRef<Held>({ ...ZERO_HELD });
  const [, setRenderTick] = useState(0); // force re-render on held state change
  const [latestState, setLatestState] = useState<AugerStateMessage | undefined>();
  const [renderDone, setRenderDone] = useState<() => void>();

  const subscriptions = useMemo(() => [{ topic: Auger.stateTopic }], []);

  // ---- subscribe to /astrotech/auger/state ----
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frames = renderState.currentFrame ?? [];
      for (const frame of frames) {
        if (frame.topic === Auger.stateTopic) {
          setLatestState(frame.message as AugerStateMessage);
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

  // ---- advertise /astrotech/auger/cmd_vel ----
  useEffect(() => {
    context.advertise?.(Auger.cmdTopic, Auger.cmdSchemaName);
    return () => {
      // Best-effort: send one zero command before unmount.
      try {
        context.publish?.(Auger.cmdTopic, ZERO_CMD);
      } catch {
        /* publishing on a torn-down panel is fine to ignore */
      }
      context.unadvertise?.(Auger.cmdTopic);
    };
  }, [context]);

  // ---- 10 Hz publish loop while any button held ----
  useEffect(() => {
    const periodMs = 1000 / Auger.publishHz;
    const interval = setInterval(() => {
      const held = heldRef.current;
      const anyHeld =
        held.up || held.down || held.spinFwd || held.spinBack;
      if (!anyHeld) {
        return; // release-edge is handled by the button handler itself
      }
      try {
        context.publish?.(Auger.cmdTopic, commandFromHeld(held));
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[urc.auger_control] publish failed", err);
      }
    }, periodMs);
    return () => clearInterval(interval);
  }, [context]);

  const press = useCallback(
    (key: keyof Held) => () => {
      if (heldRef.current[key]) return;
      heldRef.current = { ...heldRef.current, [key]: true };
      setRenderTick((n) => n + 1);
      try {
        context.publish?.(Auger.cmdTopic, commandFromHeld(heldRef.current));
      } catch {
        /* swallow — the interval will retry */
      }
    },
    [context],
  );

  const release = useCallback(
    (key: keyof Held) => () => {
      if (!heldRef.current[key]) return;
      heldRef.current = { ...heldRef.current, [key]: false };
      setRenderTick((n) => n + 1);
      const stillHeld =
        heldRef.current.up ||
        heldRef.current.down ||
        heldRef.current.spinFwd ||
        heldRef.current.spinBack;
      try {
        context.publish?.(
          Auger.cmdTopic,
          stillHeld ? commandFromHeld(heldRef.current) : ZERO_CMD,
        );
      } catch {
        /* same as above */
      }
    },
    [context],
  );

  // Defensive: if a button receives a mouseleave while held, treat as release.
  const releaseAllOnLeave = useCallback(() => {
    const wasHeld =
      heldRef.current.up ||
      heldRef.current.down ||
      heldRef.current.spinFwd ||
      heldRef.current.spinBack;
    if (!wasHeld) return;
    heldRef.current = { ...ZERO_HELD };
    setRenderTick((n) => n + 1);
    try {
      context.publish?.(Auger.cmdTopic, ZERO_CMD);
    } catch {
      /* swallow */
    }
  }, [context]);

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
      onMouseLeave={releaseAllOnLeave}
    >
      <h3 style={{ marginTop: 0, marginBottom: 4 }}>Auger control</h3>
      <div style={{ opacity: 0.7, marginBottom: 12, fontSize: 12 }}>
        Hold a button to drive. Releasing sends a zero command immediately.
      </div>

      <div
        style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr",
          gap: 8,
          marginBottom: 16,
        }}
      >
        <HoldButton
          label="Up"
          held={heldRef.current.up}
          onPress={press("up")}
          onRelease={release("up")}
        />
        <HoldButton
          label="Spin Forward"
          held={heldRef.current.spinFwd}
          onPress={press("spinFwd")}
          onRelease={release("spinFwd")}
        />
        <HoldButton
          label="Down"
          held={heldRef.current.down}
          onPress={press("down")}
          onRelease={release("down")}
        />
        <HoldButton
          label="Spin Back"
          held={heldRef.current.spinBack}
          onPress={press("spinBack")}
          onRelease={release("spinBack")}
        />
      </div>

      <Telemetry state={latestState} />
    </div>
  );
}

function HoldButton(props: {
  label: string;
  held: boolean;
  onPress: () => void;
  onRelease: () => void;
}): ReactElement {
  return (
    <button
      type="button"
      onMouseDown={props.onPress}
      onMouseUp={props.onRelease}
      onMouseLeave={props.onRelease}
      onTouchStart={(e) => {
        e.preventDefault();
        props.onPress();
      }}
      onTouchEnd={(e) => {
        e.preventDefault();
        props.onRelease();
      }}
      onTouchCancel={(e) => {
        e.preventDefault();
        props.onRelease();
      }}
      style={{
        padding: "10px 12px",
        background: props.held
          ? "var(--foxglove-accent, #5469d4)"
          : "rgba(255,255,255,0.08)",
        color: props.held ? "#fff" : "var(--foxglove-text, #e8e8e8)",
        border: "1px solid rgba(255,255,255,0.15)",
        borderRadius: 4,
        cursor: "pointer",
        userSelect: "none",
        fontSize: 13,
      }}
    >
      {props.label}
      {props.held ? " ●" : ""}
    </button>
  );
}

function Telemetry(props: { state?: AugerStateMessage }): ReactElement {
  const s = props.state;
  return (
    <div
      style={{
        borderTop: "1px solid rgba(255,255,255,0.12)",
        paddingTop: 8,
        fontSize: 12,
        lineHeight: 1.6,
      }}
    >
      <div style={{ fontWeight: 600, marginBottom: 4 }}>Telemetry</div>
      {!s && <div style={{ opacity: 0.6 }}>no /astrotech/auger/state yet</div>}
      {s && (
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "max-content 1fr 1fr",
            columnGap: 12,
            rowGap: 2,
          }}
        >
          <div />
          <div style={{ fontWeight: 600 }}>lead_screw</div>
          <div style={{ fontWeight: 600 }}>auger</div>

          <div>position (rev)</div>
          <div>{fmt(s.lead_screw_position_rev)}</div>
          <div>{fmt(s.auger_position_rev)}</div>

          <div>velocity (rev/s)</div>
          <div>{fmt(s.lead_screw_velocity_rev_s)}</div>
          <div>{fmt(s.auger_velocity_rev_s)}</div>

          <div>torque (N·m)</div>
          <div>{fmt(s.lead_screw_torque_nm)}</div>
          <div>{fmt(s.auger_torque_nm)}</div>

          <div>temp (°C)</div>
          <div>{fmt(s.lead_screw_temperature_c, 1)}</div>
          <div>{fmt(s.auger_temperature_c, 1)}</div>

          <div>mode / fault</div>
          <div>
            {s.lead_screw_mode} / {s.lead_screw_fault}
          </div>
          <div>
            {s.auger_mode} / {s.auger_fault}
          </div>
        </div>
      )}
    </div>
  );
}

export function initAugerControlPanel(
  context: PanelExtensionContext,
): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AugerPanel context={context} />);
  return () => {
    root.unmount();
  };
}
