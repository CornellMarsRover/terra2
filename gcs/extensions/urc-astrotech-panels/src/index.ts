import { ExtensionContext } from "@foxglove/extension";

import { initAnalysisSequencePanel } from "./panels/AnalysisSequence";
import { initAugerControlPanel } from "./panels/AugerControl";
import { initMixingServoPanel } from "./panels/MixingServo";
import { initRamanSpectrumPanel } from "./panels/RamanSpectrum";

// Foxglove stores panels in its registry under the key
//   `${qualifiedName}.${panel.name}`
// where, for *locally-installed* extensions (anything installed via
// `foxglove-extension install`, including this one during dev),
// `qualifiedName` resolves to the package.json `displayName` field
// VERBATIM — spaces and all. Verified in the Foxglove 2.39 renderer:
//
//   function he(r, e) {
//     switch (r) {
//       case "local": return e.displayName;
//       case "org":   return [r, qW(e), e.name].join(":");
//     }
//   }
//
// So with displayName = "URC Astrotech Panels", the four panel-type IDs
// that gcs/layouts/urc_astrotech_dashboard.json must use are:
//   "URC Astrotech Panels.urc.auger_control"
//   "URC Astrotech Panels.urc.analysis_sequence"
//   "URC Astrotech Panels.urc.mixing_servo"
//   "URC Astrotech Panels.urc.raman_spectrum"
//
// Note: if/when we publish this as a Foxglove org extension instead of
// loading locally, the prefix changes to "org:<publisher>:<name>" and
// the layout will need updating in lockstep. (The `org:` form has no
// spaces, which is one reason to prefer it for distribution.)
export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "urc.auger_control",
    initPanel: initAugerControlPanel,
  });
  extensionContext.registerPanel({
    name: "urc.analysis_sequence",
    initPanel: initAnalysisSequencePanel,
  });
  extensionContext.registerPanel({
    name: "urc.mixing_servo",
    initPanel: initMixingServoPanel,
  });
  extensionContext.registerPanel({
    name: "urc.raman_spectrum",
    initPanel: initRamanSpectrumPanel,
  });
}
