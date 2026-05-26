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

import { Raman } from "../interfaces";

// Real wavenumber-vs-intensity plot. Foxglove's built-in Plot is
// time-series-only, so the spectrum (one X array + one Y array per
// message) needs a custom canvas. The save-snapshot button is added
// alongside the rover-side snapshot service (see SnapshotControls).

interface Spectrum {
  wavenumbers: number[];
  intensities: number[];
  count: number;
  at: string;
}

interface RamanMessage {
  wavenumbers_cm_inv?: number[] | Float32Array;
  intensities?: number[] | Float32Array;
}

function toArray(a: number[] | Float32Array | undefined): number[] {
  if (!a) return [];
  return Array.from(a);
}

function RamanPanel(props: { context: PanelExtensionContext }): ReactElement {
  const { context } = props;
  const [spectrum, setSpectrum] = useState<Spectrum | undefined>();
  const [renderDone, setRenderDone] = useState<() => void>();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState<{ w: number; h: number }>({ w: 480, h: 260 });

  const subscriptions = useMemo(() => [{ topic: Raman.topic }], []);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frames = renderState.currentFrame ?? [];
      for (const frame of frames) {
        if (frame.topic !== Raman.topic) continue;
        const msg = frame.message as RamanMessage;
        const intensities = toArray(msg.intensities);
        const wavenumbers = toArray(msg.wavenumbers_cm_inv);
        if (intensities.length === 0) continue;
        setSpectrum({
          wavenumbers,
          intensities,
          count: intensities.length,
          at: new Date().toLocaleTimeString(),
        });
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

  // Track the drawing area size so the plot fills the panel and stays crisp.
  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      const r = entries[0]?.contentRect;
      if (r && r.width > 0 && r.height > 0) {
        setSize({ w: Math.floor(r.width), h: Math.floor(r.height) });
      }
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;
    const w = size.w;
    const h = size.h;
    canvas.width = Math.floor(w * dpr);
    canvas.height = Math.floor(h * dpr);
    canvas.style.width = `${w}px`;
    canvas.style.height = `${h}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);

    const text = "#c8c8c8";
    const grid = "rgba(255,255,255,0.12)";
    const line = "#5fa8ff";

    const mL = 52, mR = 12, mT = 12, mB = 30;
    const plotW = Math.max(1, w - mL - mR);
    const plotH = Math.max(1, h - mT - mB);

    if (!spectrum || spectrum.intensities.length === 0) {
      ctx.fillStyle = text;
      ctx.font = "13px sans-serif";
      ctx.fillText("waiting for spectrum…", mL, mT + plotH / 2);
      return;
    }

    const ys = spectrum.intensities;
    const n = ys.length;
    // X axis: use the wavenumber array if present, else the sample index.
    const haveX = spectrum.wavenumbers.length === n;
    const xs = haveX ? spectrum.wavenumbers : ys.map((_, i) => i);
    const xMin = xs[0] ?? 0;
    const xMax = xs[n - 1] ?? n - 1;
    let yMin = Infinity, yMax = -Infinity;
    for (const v of ys) { if (v < yMin) yMin = v; if (v > yMax) yMax = v; }
    if (!isFinite(yMin) || !isFinite(yMax)) { yMin = 0; yMax = 1; }
    if (yMax - yMin < 1e-9) { yMax = yMin + 1; }
    const yPad = (yMax - yMin) * 0.08;
    yMin -= yPad; yMax += yPad;

    const xToPx = (x: number) =>
      mL + ((x - xMin) / (xMax - xMin || 1)) * plotW;
    const yToPx = (y: number) =>
      mT + plotH - ((y - yMin) / (yMax - yMin || 1)) * plotH;

    // axes
    ctx.strokeStyle = grid;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(mL, mT); ctx.lineTo(mL, mT + plotH); ctx.lineTo(mL + plotW, mT + plotH);
    ctx.stroke();

    ctx.fillStyle = text;
    ctx.font = "10px sans-serif";
    // x ticks
    const xTicks = 5;
    ctx.textAlign = "center";
    for (let i = 0; i <= xTicks; i++) {
      const x = xMin + ((xMax - xMin) * i) / xTicks;
      const px = xToPx(x);
      ctx.fillText(x.toFixed(0), px, mT + plotH + 14);
    }
    // y ticks
    const yTicks = 4;
    ctx.textAlign = "right";
    for (let i = 0; i <= yTicks; i++) {
      const y = yMin + ((yMax - yMin) * i) / yTicks;
      const py = yToPx(y);
      ctx.fillText(y.toFixed(2), mL - 6, py + 3);
    }

    // axis labels
    ctx.textAlign = "center";
    ctx.fillText(
      haveX ? "Raman shift (cm⁻¹)" : "pixel index",
      mL + plotW / 2,
      h - 2,
    );

    // the trace
    ctx.strokeStyle = line;
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const px = xToPx(xs[i] ?? i);
      const py = yToPx(ys[i] ?? 0);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }, [size, spectrum]);

  useEffect(() => { draw(); }, [draw]);

  return (
    <div
      style={{
        padding: "10px 12px",
        fontFamily: "sans-serif",
        fontSize: 13,
        color: "var(--foxglove-text, #e8e8e8)",
        height: "100%",
        boxSizing: "border-box",
        display: "flex",
        flexDirection: "column",
      }}
    >
      <div style={{ display: "flex", alignItems: "baseline", justifyContent: "space-between" }}>
        <h3 style={{ margin: 0 }}>Raman spectrum</h3>
        <span style={{ fontSize: 11, opacity: 0.6 }}>
          {spectrum ? `${spectrum.count} pts · ${spectrum.at}` : "no data"}
        </span>
      </div>
      <div ref={wrapRef} style={{ flex: 1, minHeight: 120, marginTop: 8 }}>
        <canvas ref={canvasRef} />
      </div>
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
