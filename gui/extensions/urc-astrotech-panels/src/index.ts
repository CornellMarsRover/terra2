import { ExtensionContext } from "@foxglove/extension";

import { initAnalysisSequencePanel } from "./panels/AnalysisSequence";
import { initAugerControlPanel } from "./panels/AugerControl";
import { initMixingServoPanel } from "./panels/MixingServo";
import { initRamanSpectrumPanel } from "./panels/RamanSpectrum";
import { initSnapshotsPanel } from "./panels/Snapshots";

// Foxglove stores panel types as `${extensionId}.${panel.name}`.
//
// For locally-installed extensions (`foxglove-extension install`), the
// extension id is **`publisher` + `.` + `name`**, both normalized: lowercased,
// and the publisher has all non-alphanumeric characters stripped. This
// matches `getPackageId()` in create-foxglove-extension (and the on-disk
// folder name, minus the `-<version>` suffix).
//
// With `publisher: "cornell-mars-rover"` and `name: "urc-astrotech-panels"`:
//   extensionId === "cornellmarsrover.urc-astrotech-panels"
//
// The panel-type IDs that gui/layouts/urc_astrotech_{auger,analysis}.json
// must use are therefore:
//   "cornellmarsrover.urc-astrotech-panels.urc.auger_control"
//   "cornellmarsrover.urc-astrotech-panels.urc.analysis_sequence"
//   "cornellmarsrover.urc-astrotech-panels.urc.mixing_servo"
//   "cornellmarsrover.urc-astrotech-panels.urc.raman_spectrum"
//   "cornellmarsrover.urc-astrotech-panels.urc.snapshots"
//
// (`displayName` is human-facing UI only — do not use it in layout JSON.)
//
// If this extension is ever loaded from the Foxglove registry instead of
// `local-install`, the prefix may become `org:<publisher>:<name>` instead;
// update layouts in lockstep.
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
