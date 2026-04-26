# State for the moteus-driven auger.
# lead_screw (id=15): vertical motion. auger (id=16): rotation.
#
# These fields are real closed-loop telemetry: moteus controllers
# expose position/velocity/torque/temperature/mode/fault on every
# cycle, unlike the BDC and servo boards on Caitlin's library which
# stay open-loop.

builtin_interfaces/Time stamp

# Lead screw
float32 lead_screw_position_rev
float32 lead_screw_velocity_rev_s
float32 lead_screw_torque_Nm
float32 lead_screw_temperature_C
uint8   lead_screw_mode
uint8   lead_screw_fault

# Auger
float32 auger_position_rev
float32 auger_velocity_rev_s
float32 auger_torque_Nm
float32 auger_temperature_C
uint8   auger_mode
uint8   auger_fault
