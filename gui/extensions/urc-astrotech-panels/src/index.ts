import { ExtensionContext } from "@foxglove/extension";

import { initAnalysisSequencePanel } from "./panels/AnalysisSequence";
import { initAugerControlPanel } from "./panels/AugerControl";
import { initMixingServoPanel } from "./panels/MixingServo";
import { initRamanSpectrumPanel } from "./panels/RamanSpectrum";
import { initSnapshotsPanel } from "./panels/Snapshots";

// Foxglove stores panel types as `${extensionDisplayName}.${panel.name}` for
// these locally-installed panels. Keep gui/layouts/urc_astrotech_*.json in
// lockstep with this package's displayName:
//   "URC Astrotech Panels.urc.auger_control"
//   "URC Astrotech Panels.urc.analysis_sequence"
//   "URC Astrotech Panels.urc.mixing_servo"
//   "URC Astrotech Panels.urc.raman_spectrum"
//   "URC Astrotech Panels.urc.snapshots"
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
  extensionContext.registerPanel({
    name: "urc.snapshots",
    initPanel: initSnapshotsPanel,
  });
}
