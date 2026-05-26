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

import { MixingServo } from "../interfaces";

// ---------------------------------------------------------------------------
// Astrotech mixing-chamber operator workflow (set 2026-05-08 after the
// servo bench bring-up):
//
//   Set Home            — taps `/astrotech/mixing_servo/set_home`. The
//                         servo board reassigns its current physical
//                         position to 0°. Press once after manually
//                         walking the chamber to the "one-side" reference
//                         (using mixing_servo_min.py or this panel's
//                         manual-goto field). All later positions are
//                         offsets from there.
//
//   Big Box / Site 1 /  — single-tap calls `/astrotech/mixing_servo/set_angle`
//   Site 2                with the saved angle for that button. Defaults
//                         baked in below come from the bench session
//                         (110 / 145 / 185); edit in the "Edit presets"
//                         row to override per-rig and the panel persists
//                         the new values via `context.saveState`.
//
//   Manual goto         — type any 0..4095 (12-bit wire range), click
//                         "Goto" to send `set_angle` with that value.
//                         Useful for finding new positions or sanity
//                         pokes.
//
// The driver holds the chamber for `move_settle_s` (default 5 s) before
// returning the service response, so each button is naturally disabled
// while the previous call is in flight.
// ---------------------------------------------------------------------------

const DEFAULT_BIG_BOX_DEG = 110;
const DEFAULT_SITE_1_DEG = 145;
const DEFAULT_SITE_2_DEG = 185;

interface PanelState {
  bigBoxDeg: number;
  site1Deg: number;
  site2Deg: number;
}

const DEFAULT_STATE: PanelState = {
  bigBoxDeg: DEFAULT_BIG_BOX_DEG,
  site1Deg: DEFAULT_SITE_1_DEG,
  site2Deg: DEFAULT_SITE_2_DEG,
};

// Wire field is 12-bit unsigned. Mechanical envelope is much smaller, but
// "much smaller" is rig-specific and we'd rather refuse silly numbers in
// the panel than have the driver round-trip a service error.
const MIN_DEG = 0;
const MAX_DEG = 4095;

interface MixingServoStateMessage {
  data: number;
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

function clampInt(n: number): number {
  if (!Number.isFinite(n)) return MIN_DEG;
  return Math.max(MIN_DEG, Math.min(MAX_DEG, Math.round(n)));
}

function MixingServoPanel(props: { context: PanelExtensionContext }): ReactElement {
  const { context } = props;

  const [state, setState] = useState<PanelState>(() => {
    const init = (context.initialState as Partial<PanelState> | undefined) ?? {};
    return { ...DEFAULT_STATE, ...init };
  });

  const [latestAngle, setLatestAngle] = useState<number | undefined>();
  const [busyLabel, setBusyLabel] = useState<string | undefined>();
  const [lastResult, setLastResult] = useState<CallResult | undefined>();
  const [renderDone, setRenderDone] = useState<() => void>();
  const [manualDeg, setManualDeg] = useState<string>("0");

  // Persist the three preset angles across panel reloads.
  useEffect(() => {
    context.saveState(state);
  }, [context, state]);

  // Subscribe to current-angle state.
  const subscriptions = useMemo(
    () => [{ topic: MixingServo.stateTopic }],
    [],
  );

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frames = renderState.currentFrame ?? [];
      for (const frame of frames) {
        if (frame.topic === MixingServo.stateTopic) {
          const msg = frame.message as MixingServoStateMessage;
          if (typeof msg.data === "number" && Number.isFinite(msg.data)) {
            setLatestAngle(msg.data);
          }
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

  const callSetAngle = useCallback(
    (deg: number, label: string) => async () => {
      if (busyLabel !== undefined) return;
      const cleanDeg = clampInt(deg);
      setBusyLabel(label);
      setLastResult(undefined);
      try {
        const resp = (await context.callService?.(
          MixingServo.setAngleService,
          { angle_deg: cleanDeg },
        )) as ServiceResponse | undefined;
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

  const callSetHome = useCallback(async () => {
    if (busyLabel !== undefined) return;
    setBusyLabel("Set Home");
    setLastResult(undefined);
    try {
      const resp = (await context.callService?.(
        MixingServo.setHomeService,
        {},
      )) as ServiceResponse | undefined;
      setLastResult({
        ok: !!resp?.success,
        msg: resp?.message ?? "(no response)",
        label: "Set Home",
      });
    } catch (err) {
      setLastResult({ ok: false, msg: String(err), label: "Set Home" });
    } finally {
      setBusyLabel(undefined);
    }
  }, [context, busyLabel]);

  const updatePreset = useCallback(
    (key: keyof PanelState) => (raw: string) => {
      const parsed = parseInt(raw, 10);
      if (!Number.isFinite(parsed)) return;
      setState((prev) => ({ ...prev, [key]: clampInt(parsed) }));
    },
    [],
  );

  const manualNum = parseInt(manualDeg, 10);
  const manualValid = Number.isFinite(manualNum);

  const haveCallService = typeof context.callService === "function";
  const disabled = !haveCallService || busyLabel !== undefined;

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
      <h3 style={{ marginTop: 0, marginBottom: 4 }}>Mixing servo</h3>
      <div style={{ opacity: 0.7, marginBottom: 12, fontSize: 12 }}>
        Set Home zeros the chamber at its current physical position. The
        three preset buttons drive to the saved offsets; edit the numbers
        below to update them. Each call holds for ~5 s while the chamber
        moves before the button re-enables.
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
          The current data source does not support service calls. Connect
          via the Foxglove WebSocket bridge (foxglove_bridge) to use this
          panel.
        </div>
      )}

      <Group label="Home">
        <ActionButton
          label="Set Home (current = 0°)"
          subtitle="Sends set_home(0). Press after manually walking to the reference position."
          active={busyLabel === "Set Home"}
          disabled={disabled}
          onClick={callSetHome}
          fullWidth
        />
      </Group>

      <Group label="Site presets">
        <ActionButton
          label={`Big Box  →  ${state.bigBoxDeg}°`}
          subtitle="position 0"
          active={busyLabel === "Big Box"}
          disabled={disabled}
          onClick={callSetAngle(state.bigBoxDeg, "Big Box")}
        />
        <ActionButton
          label={`Site 1  →  ${state.site1Deg}°`}
          active={busyLabel === "Site 1"}
          disabled={disabled}
          onClick={callSetAngle(state.site1Deg, "Site 1")}
        />
        <ActionButton
          label={`Site 2  →  ${state.site2Deg}°`}
          active={busyLabel === "Site 2"}
          disabled={disabled}
          onClick={callSetAngle(state.site2Deg, "Site 2")}
        />
      </Group>

      <Group label="Manual goto">
        <div
          style={{
            gridColumn: "1 / -1",
            display: "grid",
            gridTemplateColumns: "1fr auto",
            gap: 8,
            alignItems: "stretch",
          }}
        >
          <NumberInput
            value={manualDeg}
            onChange={setManualDeg}
            disabled={disabled}
            placeholder="degrees"
          />
          <ActionButton
            label={manualValid ? `Goto ${clampInt(manualNum)}°` : "Goto …"}
            active={busyLabel === "Manual"}
            disabled={disabled || !manualValid}
            onClick={callSetAngle(manualValid ? manualNum : 0, "Manual")}
          />
        </div>
      </Group>

      <Group label="Edit presets">
        <PresetEditor
          label="Big Box"
          value={state.bigBoxDeg}
          onChange={updatePreset("bigBoxDeg")}
          disabled={busyLabel !== undefined}
        />
        <PresetEditor
          label="Site 1"
          value={state.site1Deg}
          onChange={updatePreset("site1Deg")}
          disabled={busyLabel !== undefined}
        />
        <PresetEditor
          label="Site 2"
          value={state.site2Deg}
          onChange={updatePreset("site2Deg")}
          disabled={busyLabel !== undefined}
        />
      </Group>

      <Telemetry
        latestAngle={latestAngle}
        busyLabel={busyLabel}
        lastResult={lastResult}
      />
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
      <div
        style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr",
          gap: 8,
        }}
      >
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
  fullWidth?: boolean;
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
        gridColumn: props.fullWidth ? "1 / -1" : undefined,
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
        {props.active ? " ●" : ""}
      </div>
      {props.subtitle && (
        <div style={{ fontSize: 11, opacity: 0.65, marginTop: 2 }}>
          {props.subtitle}
        </div>
      )}
    </button>
  );
}

function NumberInput(props: {
  value: string;
  onChange: (v: string) => void;
  disabled?: boolean;
  placeholder?: string;
}): ReactElement {
  return (
    <input
      type="number"
      step={1}
      min={MIN_DEG}
      max={MAX_DEG}
      value={props.value}
      placeholder={props.placeholder}
      disabled={props.disabled}
      onChange={(e) => props.onChange(e.target.value)}
      style={{
        background: "rgba(255,255,255,0.06)",
        color: "var(--foxglove-text, #e8e8e8)",
        border: "1px solid rgba(255,255,255,0.15)",
        borderRadius: 4,
        padding: "10px 10px",
        fontSize: 13,
        fontFamily: "inherit",
        minWidth: 0,
      }}
    />
  );
}

function PresetEditor(props: {
  label: string;
  value: number;
  onChange: (v: string) => void;
  disabled?: boolean;
}): ReactElement {
  return (
    <label
      style={{
        display: "grid",
        gridTemplateColumns: "max-content 1fr",
        alignItems: "center",
        gap: 8,
        fontSize: 12,
        opacity: props.disabled ? 0.7 : 1,
      }}
    >
      <span>{props.label}</span>
      <input
        type="number"
        step={1}
        min={MIN_DEG}
        max={MAX_DEG}
        value={String(props.value)}
        disabled={props.disabled}
        onChange={(e) => props.onChange(e.target.value)}
        style={{
          background: "rgba(255,255,255,0.06)",
          color: "var(--foxglove-text, #e8e8e8)",
          border: "1px solid rgba(255,255,255,0.15)",
          borderRadius: 4,
          padding: "6px 8px",
          fontSize: 12,
          fontFamily: "inherit",
          minWidth: 0,
        }}
      />
    </label>
  );
}

function Telemetry(props: {
  latestAngle: number | undefined;
  busyLabel: string | undefined;
  lastResult: CallResult | undefined;
}): ReactElement {
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
      <div>
        last commanded angle:{" "}
        <code>
          {props.latestAngle === undefined ? "—" : `${props.latestAngle}°`}
        </code>
      </div>
      <div>
        status:{" "}
        {props.busyLabel ? (
          <span style={{ color: "var(--foxglove-accent, #5469d4)" }}>
            calling {props.busyLabel}…
          </span>
        ) : (
          <span style={{ opacity: 0.7 }}>idle</span>
        )}
      </div>
      {props.lastResult && (
        <div
          style={{
            marginTop: 4,
            color: props.lastResult.ok ? "#7fcf7f" : "#e57373",
          }}
        >
          {props.lastResult.label}: {props.lastResult.ok ? "OK" : "ERR"}
          {props.lastResult.msg ? ` — ${props.lastResult.msg}` : ""}
        </div>
      )}
    </div>
  );
}

export function initMixingServoPanel(
  context: PanelExtensionContext,
): () => void {
  const root = createRoot(context.panelElement);
  root.render(<MixingServoPanel context={context} />);
  return () => {
    root.unmount();
  };
}
