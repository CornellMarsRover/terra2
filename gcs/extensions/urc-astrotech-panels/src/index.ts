import { ExtensionContext } from "@foxglove/extension";

import { initAnalysisSequencePanel } from "./panels/AnalysisSequence";
import { initAugerControlPanel } from "./panels/AugerControl";
import { initMixingServoPanel } from "./panels/MixingServo";
import { initRamanSpectrumPanel } from "./panels/RamanSpectrum";

// Panel-type strings registered here MUST match the types used in
// gcs/layouts/urc_astrotech_dashboard.json:
//   "urc.auger_control", "urc.analysis_sequence",
//   "urc.mixing_servo",  "urc.raman_spectrum".
//
// If you rename them, update the layout JSON at the same time.
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
