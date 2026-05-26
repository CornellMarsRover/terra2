// Single source of truth for topic/service/action names on the Foxglove side.
// MUST stay in sync with src/astrotech_rover/config/astrotech_interfaces.yaml.
// The YAML is authoritative; this file is a hand-written mirror of it.
//
// When the YAML changes, update this file and re-run `npm run build`.
//
// Why not parse the YAML at runtime? Browser-side YAML parsing would pull
// in a dep just to avoid ~50 lines of copy-paste. Not worth it.

// Auger lives on two moteus controllers (id=15 lead_screw, id=16 auger),
// distinct from the BDC/Servo CAN-FD stack that drives the pumps. Hold-to-act
// command pattern: GCS publishes AugerCommand at 10 Hz while a button is held
// and once with all zeros on release. Driver applies a 200 ms watchdog.
export const Auger = {
  cmdTopic: "/astrotech/auger/cmd_vel",
  cmdType: "cmr_msgs/AugerCommand",
  cmdSchemaName: "cmr_msgs/msg/AugerCommand",
  stateTopic: "/astrotech/auger/state",
  stateType: "cmr_msgs/AugerState",
  publishHz: 10,
  defaults: {
    leadScrewVelRevS: 100.0,
    leadScrewMaxTorqueNm: 2.0,
    augerSpinForwardRevS: 10.0,
    augerSpinBackwardRevS: 50.0, // matches asymmetric values in the test harness
    augerMaxTorqueNm: 1.0,
  },
} as const;

// CMR servo board on the fdcanusb (can_id=26, servo_id=15). Two services:
//   - setAngle: drive to N° (offset from the most recently set home).
//   - setHome:  declare current physical position to be 0°.
// State topic carries the last commanded angle (Int32). The panel keeps
// the three site offsets in its own saved state and calls setAngle with
// whichever value is in the editor for the button that was clicked.
// See gui/README.md ("Bench-discovered facts") for the history.
export const MixingServo = {
  setAngleService: "/astrotech/mixing_servo/set_angle",
  setAngleType: "cmr_msgs/SetMixingServoAngle",
  setAngleSchemaName: "cmr_msgs/srv/SetMixingServoAngle",
  setHomeService: "/astrotech/mixing_servo/set_home",
  setHomeType: "std_srvs/Trigger",
  setHomeSchemaName: "std_srvs/srv/Trigger",
  stateTopic: "/astrotech/mixing_servo/state",
  stateType: "std_msgs/Int32",
} as const;

// TODO: two action servers, one per
// sequence id. If sequence_id gets renamed to site_num (or similar), update
// both here and the .action definition.
export const Analysis = {
  actionType: "cmr_msgs/RunAnalysisSequence",
  sequences: [
    { id: 1, action: "/astrotech/analysis/run_sequence_1" },
    { id: 2, action: "/astrotech/analysis/run_sequence_2" },
  ],
} as const;

// TODO: replace if a real Raman driver publishes a
// different message type.
export const Raman = {
  topic: "/astrotech/raman/spectrum",
  type: "cmr_msgs/RamanSpectrum",
} as const;

// TODO: same as Q5.
export const Env = {
  topic: "/astrotech/env/sample",
  type: "cmr_msgs/EnvSample",
} as const;

// Save-plot-to-disk services, implemented by the snapshot driver. The
// driver renders a PNG and writes it to the laptop Desktop (fallback:
// <repo>/snapshots/), returning the saved path in the Trigger response.
export const Snapshot = {
  saveRamanService: "/astrotech/snapshot/raman",
  saveCo2Service: "/astrotech/snapshot/co2",
  type: "std_srvs/Trigger",
  schemaName: "std_srvs/srv/Trigger",
} as const;

// Camera feeds: launched rover-side via src/cmr_cams/ (cmr_cv camera
// nodes for /camN/image_raw + stereolabs zed_wrapper for /zed/...).
// Foxglove uses its built-in Image panel referencing those topics
// directly from the layout JSONs; the extension does not own a camera
// registry. See gui/operator_guide.md for the four-terminal bring-up.
