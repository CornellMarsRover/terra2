// Single source of truth for topic/service/action names on the Foxglove side.
// MUST stay in sync with src/urc_mock_rover/config/astrotech_interfaces.yaml.
// The YAML is authoritative; this file is a hand-written mirror of it.
//
// When the YAML changes, update this file and re-run `npm run build`.
//
// Why not parse the YAML at runtime? Browser-side YAML parsing would pull
// in a dep just to avoid ~50 lines of copy-paste. Not worth it.

// TODO(astrotech-q-1): auger command topic + type. If the real driver uses
// something other than geometry_msgs/Twist, change `Auger.cmdType` and the
// panel that publishes to it.
export const Auger = {
  cmdTopic: "/astrotech/auger/cmd",
  cmdType: "geometry_msgs/Twist",
  stateTopic: "/astrotech/auger/state",
  stateType: "cmr_msgs/AugerState",
} as const;

// TODO(astrotech-q-2): mixing servo controller family is unknown. Service
// contract below is the assumption.
export const MixingServo = {
  setPresetService: "/astrotech/mixing_servo/set_preset",
  setPresetType: "cmr_msgs/SetMixingServoPreset",
  stateTopic: "/astrotech/mixing_servo/state",
  stateType: "std_msgs/String",
  presets: ["S1", "S2", "CO2_1", "CO2_2", "RETRACT"] as const,
} as const;

// TODO(astrotech-q-3), TODO(astrotech-q-4): two action servers, one per
// sequence id. If sequence_id gets renamed to site_num (or similar), update
// both here and the .action definition.
export const Analysis = {
  actionType: "cmr_msgs/RunAnalysisSequence",
  sequences: [
    { id: 1, action: "/astrotech/analysis/run_sequence_1" },
    { id: 2, action: "/astrotech/analysis/run_sequence_2" },
  ],
} as const;

// TODO(astrotech-q-5): replace if a real Raman driver publishes a
// different message type.
export const Raman = {
  topic: "/astrotech/raman/spectrum",
  type: "cmr_msgs/RamanSpectrum",
} as const;

// TODO(astrotech-q-6): same as Q5.
export const Env = {
  topic: "/astrotech/env/sample",
  type: "cmr_msgs/EnvSample",
} as const;

// Camera feeds: handled on a separate branch and merged in later.
// Foxglove uses its built-in Image panel for whichever topics that
// branch exposes; the extension does not own a camera registry today.

export type MixingServoPreset = (typeof MixingServo.presets)[number];
