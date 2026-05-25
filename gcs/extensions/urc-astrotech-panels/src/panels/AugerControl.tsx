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

// ---------------------------------------------------------------------------
// Astrotech operator workflow (set by the team on 2026-04-30, revised
// 2026-05-08 after auger hardware bring-up):
//
//   Up no spin           — hold to retract the lead screw (no rotation).
//   Down + spin CCW      — drilling motion (lead screw down, auger CCW).
//   Spin CW / Spin CCW   — rotation only, no lead-screw motion. Used to
//                          clear / unjam / back the bit out.
//
// Plus four position-memory buttons + a stop:
//
//   Set Up Home          — one-tap: capture the current lead-screw position
//                          as "up home". In-memory only; resets on panel
//                          reload or rover restart (the moteus position
//                          counter is not physically anchored, so persisting
//                          across power cycles would be unsafe).
//   Set Bottom Home      — same, but for "bottom home".
//   Return to Up Home    — one-tap: drive the lead screw to the saved up
//                          home position autonomously. Disabled until "Set
//                          Up Home" has been pressed at least once.
//   Return to Bottom     — same, but for bottom home.
//   Stop                 — abort everything: clears all holds and any active
//                          Return move, then publishes a zero command.
//
// Hold-to-act behavior for the velocity buttons is unchanged: the publish
// loop sums the (leadScrewVel, augerVel) contributions of every currently
// held button so e.g. holding "Down + spin CCW" while tapping "Spin CW"
// cancels rotation but keeps drive-down going. Releasing all buttons sends
// one explicit zero command before the watchdog can fire.
//
// Returns are exclusive: only one Return can be active at a time, and
// pressing any *velocity* button cancels an active Return (manual control
// takes priority over autonomous moves). Pressing the same Return button
// while it is already moving is a no-op — use Stop to abort.
//
// Direction sign convention (mirrors `interfaces.ts` and the keyboard test
// harness at `docs/reference/auger_keys_test_harness.py`):
//   - lead screw: +leadScrewVelRevS = up, -leadScrewVelRevS = down.
//   - auger:      +augerSpinForwardRevS  = "FORWARD" in the harness,
//                 -augerSpinBackwardRevS = "BACKWARD" in the harness.
// We map FORWARD → CW and BACKWARD → CCW arbitrarily; the harness doesn't
// say which is which physically. Flip CW_DIR / CCW_DIR below if hardware
// bring-up shows the labels are inverted.
// ---------------------------------------------------------------------------

const CW_DIR = +1; // multiplier on augerSpinForwardRevS
const CCW_DIR = -1; // multiplier on augerSpinBackwardRevS magnitude

// Return-to-home uses (1) fast approach outside the slow zone,
// (2) proportional speed inside the slow zone so we slow continuously
// as error shrinks (fixed creep was still fast enough to overshoot and
// hunt), and (3) a *settle debounce* near the target: while
// |target - pos| is below RETURN_TOLERANCE_REV we command **zero** lead
// velocity from the Return. Completion needs RETURN_SETTLE_TICKS consecutive
// good samples: position inside that band, reported |lead_screw_velocity|
// below RETURN_SETTLE_MAX_VEL_REV_S, and |error| not above RETURN_SETTLE_LOST_REV.
// The velocity gate avoids counting "arrived" while the screw is bouncing on
// a hard stop at the top; position-only debounce still flips with jitter.
const RETURN_FAST_REV_S = 50.0;
const RETURN_CREEP_MAX_REV_S = 0.5;
// When |target - pos| is outside this, use RETURN_FAST_REV_S.
const RETURN_SLOW_ZONE_REV = 2.0;
// P-gain for the final approach: rev/s per rev of error, capped at CREEP_MAX.
const RETURN_SLOW_KP = 0.45;
const RETURN_SLOW_MIN_REV_S = 0.05;

// Inside this |error|, we command no lead velocity from Return and (with a
// velocity gate, below) count toward settle-complete.
const RETURN_TOLERANCE_REV = 0.52;
/** Above this |error|, reset the settle debounce (must re-approach). */
const RETURN_SETTLE_LOST_REV = 0.95;
/** Streak increments only when |Telemetry lead velocity| is below this. */
const RETURN_SETTLE_MAX_VEL_REV_S = 0.45;
// Consecutive good samples before we clear the move (~0.4–0.5 s at 10 Hz).
const RETURN_SETTLE_TICKS = 5;

/** Magnitude of lead-screw velocity (rev/s) for an active Return, when
 *  |error| is at least RETURN_TOLERANCE_REV. */
function returnApproachLeadMagnitude(absDelta: number): number {
  if (absDelta > RETURN_SLOW_ZONE_REV) {
    return RETURN_FAST_REV_S;
  }
  return Math.min(
    RETURN_CREEP_MAX_REV_S,
    Math.max(RETURN_SLOW_MIN_REV_S, absDelta * RETURN_SLOW_KP),
  );
}

// --- Velocity buttons (hold-to-act) ----------------------------------------

type VelocityActionId =
  | "upNoSpin"
  | "downSpinCcw"
  | "spinCw"
  | "spinCcw";

interface ActionSpec {
  label: string;
  subtitle?: string;
  leadVel: number;
  augerVel: number;
}

const VELOCITY_ACTIONS: Record<VelocityActionId, ActionSpec> = {
  upNoSpin: {
    label: "Up (no spin)",
    leadVel: +Auger.defaults.leadScrewVelRevS,
    augerVel: 0,
  },
  downSpinCcw: {
    label: "Down + spin CCW",
    subtitle: "drilling motion",
    leadVel: -Auger.defaults.leadScrewVelRevS,
    augerVel: CCW_DIR * Auger.defaults.augerSpinBackwardRevS,
  },
  spinCw: {
    label: "Spin CW",
    leadVel: 0,
    augerVel: CW_DIR * Auger.defaults.augerSpinForwardRevS,
  },
  spinCcw: {
    label: "Spin CCW",
    leadVel: 0,
    augerVel: CCW_DIR * Auger.defaults.augerSpinBackwardRevS,
  },
};

const ALL_VELOCITY_IDS = Object.keys(VELOCITY_ACTIONS) as VelocityActionId[];

// --- Return-to-home buttons (one-tap autonomous moves) ---------------------

type HomeKey = "up" | "bottom";

interface Homes {
  up: number | undefined;
  bottom: number | undefined;
}

const ZERO_HOMES: Homes = { up: undefined, bottom: undefined };

type ReturnId = "returnUpHome" | "returnBottomHome";

const RETURN_ACTIONS: Record<ReturnId, { label: string; homeKey: HomeKey }> = {
  returnUpHome: { label: "Return to Up Home", homeKey: "up" },
  returnBottomHome: { label: "Return to Bottom Home", homeKey: "bottom" },
};

const ALL_RETURN_IDS = Object.keys(RETURN_ACTIONS) as ReturnId[];

// --- Combined held state ---------------------------------------------------

type Held = Record<VelocityActionId | ReturnId, boolean>;

const ZERO_HELD: Held = {
  upNoSpin: false,
  downSpinCcw: false,
  spinCw: false,
  spinCcw: false,
  returnUpHome: false,
  returnBottomHome: false,
};

function anyHeld(held: Held): boolean {
  for (const id of ALL_VELOCITY_IDS) if (held[id]) return true;
  for (const id of ALL_RETURN_IDS) if (held[id]) return true;
  return false;
}

// --- Wire-format messages --------------------------------------------------

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

interface CommandFromHeldResult {
  cmd: AugerCommandMessage;
  // Returns aborted by missing telemetry / missing home (defensive only).
  reachedReturns: ReturnId[];
}

function commandFromHeld(
  held: Held,
  homes: Homes,
  state: AugerStateMessage | undefined,
): CommandFromHeldResult {
  let leadVel = 0.0;
  let augerVel = 0.0;
  const reached: ReturnId[] = [];

  for (const id of ALL_VELOCITY_IDS) {
    if (held[id]) {
      leadVel += VELOCITY_ACTIONS[id].leadVel;
      augerVel += VELOCITY_ACTIONS[id].augerVel;
    }
  }

  // Returns drive lead screw toward saved target. Velocity-button press
  // handlers already clear active returns, so in practice both sources do
  // not contribute to leadVel at the same time, but be defensive.
  for (const id of ALL_RETURN_IDS) {
    if (!held[id]) continue;
    const target = homes[RETURN_ACTIONS[id].homeKey];
    const current = state?.lead_screw_position_rev;
    if (
      target === undefined ||
      current === undefined ||
      !Number.isFinite(current)
    ) {
      // No telemetry or no home — caller should not have allowed this; abort
      // the return defensively rather than driving blind.
      reached.push(id);
      continue;
    }
    const delta = target - current;
    const absDelta = Math.abs(delta);
    // Inside tolerance: creep controller commands **zero**; the 10 Hz loop
    // uses a debounce counter (see settle streak) before declaring done.
    if (absDelta < RETURN_TOLERANCE_REV) {
      continue;
    }
    const returnMag = returnApproachLeadMagnitude(absDelta);
    leadVel += Math.sign(delta) * returnMag;
  }

  return {
    cmd: {
      lead_screw_velocity_rev_s: leadVel,
      lead_screw_max_torque_nm: Auger.defaults.leadScrewMaxTorqueNm,
      auger_velocity_rev_s: augerVel,
      auger_max_torque_nm: Auger.defaults.augerMaxTorqueNm,
    },
    reachedReturns: reached,
  };
}

/** Per-active-Return debounce: increment streak while inside the position
 *  band *and* lead-screw velocity from telemetry is low (otherwise the screw
 *  may still be coasting / bouncing off a hard stop at the top). */
function settleReturns(
  held: Held,
  homes: Homes,
  state: AugerStateMessage | undefined,
  prevStreaks: Partial<Record<ReturnId, number>>,
): { streaks: Partial<Record<ReturnId, number>>; settled: ReturnId[] } {
  const streaks: Partial<Record<ReturnId, number>> = {};
  const settled: ReturnId[] = [];
  const vel = state?.lead_screw_velocity_rev_s;
  const velOk =
    vel !== undefined &&
    Number.isFinite(vel) &&
    Math.abs(vel) < RETURN_SETTLE_MAX_VEL_REV_S;

  for (const id of ALL_RETURN_IDS) {
    if (!held[id]) {
      streaks[id] = 0;
      continue;
    }
    const target = homes[RETURN_ACTIONS[id].homeKey];
    const current = state?.lead_screw_position_rev;
    if (
      target === undefined ||
      current === undefined ||
      !Number.isFinite(current)
    ) {
      streaks[id] = 0;
      continue;
    }
    const absDelta = Math.abs(target - current);

    if (absDelta > RETURN_SETTLE_LOST_REV) {
      streaks[id] = 0;
      continue;
    }

    if (absDelta < RETURN_TOLERANCE_REV && velOk) {
      const next = (prevStreaks[id] ?? 0) + 1;
      streaks[id] = next;
      if (next >= RETURN_SETTLE_TICKS) {
        settled.push(id);
      }
    } else {
      streaks[id] = prevStreaks[id] ?? 0;
    }
  }

  return { streaks, settled };
}

function fmt(n: number | undefined, digits = 2): string {
  return typeof n === "number" && Number.isFinite(n) ? n.toFixed(digits) : "—";
}

// ---------------------------------------------------------------------------

function AugerPanel(props: { context: PanelExtensionContext }): ReactElement {
  const { context } = props;
  const heldRef = useRef<Held>({ ...ZERO_HELD });
  const homesRef = useRef<Homes>({ ...ZERO_HOMES });
  const stateRef = useRef<AugerStateMessage | undefined>(undefined);
  /** Debounce counter per Return while |error| < tolerance (see settleReturns). */
  const settleStreakRef = useRef<Partial<Record<ReturnId, number>>>({});
  const [, setRenderTick] = useState(0); // force re-render on held / homes changes
  const [latestState, setLatestState] = useState<AugerStateMessage | undefined>();
  const [renderDone, setRenderDone] = useState<() => void>();

  const subscriptions = useMemo(() => [{ topic: Auger.stateTopic }], []);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frames = renderState.currentFrame ?? [];
      for (const frame of frames) {
        if (frame.topic === Auger.stateTopic) {
          const msg = frame.message as AugerStateMessage;
          stateRef.current = msg;
          setLatestState(msg);
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

  useEffect(() => {
    context.advertise?.(Auger.cmdTopic, Auger.cmdSchemaName);
    return () => {
      try {
        context.publish?.(Auger.cmdTopic, ZERO_CMD);
      } catch {
        /* publishing on a torn-down panel is fine to ignore */
      }
      context.unadvertise?.(Auger.cmdTopic);
    };
  }, [context]);

  // 10 Hz publish loop. While anything is held, publish the computed
  // command. Return-to-home "arrived" is detected via settleReturns (debounce
  // inside tolerance), merged with defensive reaches from commandFromHeld.
  useEffect(() => {
    const periodMs = 1000 / Auger.publishHz;
    const interval = setInterval(() => {
      const held = heldRef.current;
      if (!anyHeld(held)) {
        settleStreakRef.current = {};
        return; // release-edge is handled by the button handler itself
      }

      const { streaks, settled } = settleReturns(
        held,
        homesRef.current,
        stateRef.current,
        settleStreakRef.current,
      );
      settleStreakRef.current = streaks;

      const result = commandFromHeld(
        held,
        homesRef.current,
        stateRef.current,
      );
      const mergedReached = [
        ...new Set([...result.reachedReturns, ...settled]),
      ];

      if (mergedReached.length > 0) {
        const newHeld: Held = { ...held };
        for (const id of mergedReached) {
          newHeld[id] = false;
          settleStreakRef.current[id] = 0;
        }
        heldRef.current = newHeld;
        setRenderTick((n) => n + 1);
        const survivor = anyHeld(newHeld)
          ? commandFromHeld(newHeld, homesRef.current, stateRef.current).cmd
          : ZERO_CMD;
        try {
          context.publish?.(Auger.cmdTopic, survivor);
        } catch (err) {
          // eslint-disable-next-line no-console
          console.warn("[urc.auger_control] publish failed", err);
        }
        return;
      }
      try {
        context.publish?.(Auger.cmdTopic, result.cmd);
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[urc.auger_control] publish failed", err);
      }
    }, periodMs);
    return () => clearInterval(interval);
  }, [context]);

  const pressVelocity = useCallback(
    (id: VelocityActionId) => () => {
      if (heldRef.current[id]) return;
      // Cancel any active returns -- manual control takes priority.
      const newHeld: Held = {
        ...heldRef.current,
        [id]: true,
        returnUpHome: false,
        returnBottomHome: false,
      };
      heldRef.current = newHeld;
      settleStreakRef.current = {};
      setRenderTick((n) => n + 1);
      try {
        context.publish?.(
          Auger.cmdTopic,
          commandFromHeld(newHeld, homesRef.current, stateRef.current).cmd,
        );
      } catch {
        /* swallow — the interval will retry */
      }
    },
    [context],
  );

  const releaseVelocity = useCallback(
    (id: VelocityActionId) => () => {
      if (!heldRef.current[id]) return;
      const newHeld: Held = { ...heldRef.current, [id]: false };
      heldRef.current = newHeld;
      setRenderTick((n) => n + 1);
      const stillHeld = anyHeld(newHeld);
      try {
        context.publish?.(
          Auger.cmdTopic,
          stillHeld
            ? commandFromHeld(newHeld, homesRef.current, stateRef.current).cmd
            : ZERO_CMD,
        );
      } catch {
        /* same as above */
      }
    },
    [context],
  );

  const setHome = useCallback(
    (key: HomeKey) => () => {
      const current = stateRef.current?.lead_screw_position_rev;
      if (current === undefined || !Number.isFinite(current)) {
        // Telemetry has not arrived yet -- silently ignore the press.
        return;
      }
      homesRef.current = { ...homesRef.current, [key]: current };
      setRenderTick((n) => n + 1);
    },
    [],
  );

  const startReturn = useCallback(
    (id: ReturnId) => () => {
      if (heldRef.current[id]) return; // already returning, ignore
      const homeKey = RETURN_ACTIONS[id].homeKey;
      if (homesRef.current[homeKey] === undefined) {
        return; // no home set, can't return (button is disabled in the UI too)
      }
      // Returns are mutually exclusive; clear any other in-progress return.
      const newHeld: Held = { ...heldRef.current, [id]: true };
      for (const otherId of ALL_RETURN_IDS) {
        if (otherId !== id) newHeld[otherId] = false;
      }
      heldRef.current = newHeld;
      settleStreakRef.current = {};
      setRenderTick((n) => n + 1);
      try {
        context.publish?.(
          Auger.cmdTopic,
          commandFromHeld(newHeld, homesRef.current, stateRef.current).cmd,
        );
      } catch {
        /* swallow */
      }
    },
    [context],
  );

  const stop = useCallback(() => {
    heldRef.current = { ...ZERO_HELD };
    settleStreakRef.current = {};
    setRenderTick((n) => n + 1);
    try {
      context.publish?.(Auger.cmdTopic, ZERO_CMD);
    } catch {
      /* swallow */
    }
  }, [context]);

  // Defensive: cursor leaving the panel mid-press releases velocity buttons,
  // but does NOT cancel an active Return (those are sticky one-tap states,
  // not held-by-mouse). Per-button mouseleave handles the common drag-off.
  const releaseVelocityOnLeave = useCallback(() => {
    let anyReleased = false;
    const newHeld: Held = { ...heldRef.current };
    for (const id of ALL_VELOCITY_IDS) {
      if (newHeld[id]) {
        newHeld[id] = false;
        anyReleased = true;
      }
    }
    if (!anyReleased) return;
    heldRef.current = newHeld;
    setRenderTick((n) => n + 1);
    const stillHeld = anyHeld(newHeld);
    try {
      context.publish?.(
        Auger.cmdTopic,
        stillHeld
          ? commandFromHeld(newHeld, homesRef.current, stateRef.current).cmd
          : ZERO_CMD,
      );
    } catch {
      /* swallow */
    }
  }, [context]);

  const homes = homesRef.current;
  const held = heldRef.current;
  const haveTelemetry =
    latestState !== undefined &&
    Number.isFinite(latestState.lead_screw_position_rev);

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
      onMouseLeave={releaseVelocityOnLeave}
    >
      <h3 style={{ marginTop: 0, marginBottom: 4 }}>Auger control</h3>
      <div style={{ opacity: 0.7, marginBottom: 12, fontSize: 12 }}>
        Hold velocity buttons to drive; release sends a zero command. Tap a
        Return button to drive autonomously to the saved home position; tap
        Stop (or any velocity button) to abort.
      </div>

      <ButtonGroup label="Position">
        <HoldButton
          spec={VELOCITY_ACTIONS.upNoSpin}
          held={held.upNoSpin}
          onPress={pressVelocity("upNoSpin")}
          onRelease={releaseVelocity("upNoSpin")}
          fullWidth
        />
      </ButtonGroup>

      <ButtonGroup label="Drilling">
        <HoldButton
          spec={VELOCITY_ACTIONS.downSpinCcw}
          held={held.downSpinCcw}
          onPress={pressVelocity("downSpinCcw")}
          onRelease={releaseVelocity("downSpinCcw")}
          fullWidth
        />
      </ButtonGroup>

      <ButtonGroup label="Rotation only">
        <HoldButton
          spec={VELOCITY_ACTIONS.spinCw}
          held={held.spinCw}
          onPress={pressVelocity("spinCw")}
          onRelease={releaseVelocity("spinCw")}
        />
        <HoldButton
          spec={VELOCITY_ACTIONS.spinCcw}
          held={held.spinCcw}
          onPress={pressVelocity("spinCcw")}
          onRelease={releaseVelocity("spinCcw")}
        />
      </ButtonGroup>

      <ButtonGroup label="Lead-screw home positions">
        <TapButton
          label="Set Up Home"
          subtitle={
            homes.up !== undefined
              ? `saved: ${homes.up.toFixed(3)} rev`
              : "(captures current lead-screw position)"
          }
          disabled={!haveTelemetry}
          onTap={setHome("up")}
        />
        <TapButton
          label="Return to Up Home"
          subtitle={
            homes.up !== undefined
              ? `target: ${homes.up.toFixed(3)} rev`
              : "(set Up Home first)"
          }
          disabled={homes.up === undefined || !haveTelemetry}
          active={held.returnUpHome}
          onTap={startReturn("returnUpHome")}
        />
        <TapButton
          label="Set Bottom Home"
          subtitle={
            homes.bottom !== undefined
              ? `saved: ${homes.bottom.toFixed(3)} rev`
              : "(captures current lead-screw position)"
          }
          disabled={!haveTelemetry}
          onTap={setHome("bottom")}
        />
        <TapButton
          label="Return to Bottom Home"
          subtitle={
            homes.bottom !== undefined
              ? `target: ${homes.bottom.toFixed(3)} rev`
              : "(set Bottom Home first)"
          }
          disabled={homes.bottom === undefined || !haveTelemetry}
          active={held.returnBottomHome}
          onTap={startReturn("returnBottomHome")}
        />
      </ButtonGroup>

      <ButtonGroup label="Stop">
        <StopButton onTap={stop} />
      </ButtonGroup>

      <Telemetry state={latestState} />
    </div>
  );
}

function ButtonGroup(props: {
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

function HoldButton(props: {
  spec: ActionSpec;
  held: boolean;
  onPress: () => void;
  onRelease: () => void;
  fullWidth?: boolean;
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
        gridColumn: props.fullWidth ? "1 / -1" : undefined,
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
        textAlign: "left",
        lineHeight: 1.3,
      }}
    >
      <div style={{ fontWeight: 600 }}>
        {props.spec.label}
        {props.held ? " ●" : ""}
      </div>
      {props.spec.subtitle && (
        <div style={{ fontSize: 11, opacity: 0.65, marginTop: 2 }}>
          {props.spec.subtitle}
        </div>
      )}
    </button>
  );
}

function TapButton(props: {
  label: string;
  subtitle?: string;
  disabled?: boolean;
  active?: boolean; // for Return buttons that are mid-move
  fullWidth?: boolean;
  onTap: () => void;
}): ReactElement {
  return (
    <button
      type="button"
      disabled={props.disabled}
      onClick={() => {
        if (!props.disabled) props.onTap();
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
        opacity: props.disabled ? 0.55 : 1,
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

function StopButton(props: { onTap: () => void }): ReactElement {
  return (
    <button
      type="button"
      onClick={props.onTap}
      style={{
        gridColumn: "1 / -1",
        padding: "12px 12px",
        background: "#9b3838",
        color: "#fff",
        border: "1px solid #c25151",
        borderRadius: 4,
        cursor: "pointer",
        userSelect: "none",
        fontSize: 14,
        fontWeight: 700,
        textAlign: "center",
        textTransform: "uppercase",
        letterSpacing: 0.5,
      }}
    >
      Stop
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
