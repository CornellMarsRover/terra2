# Velocity command for the moteus-driven auger.
#
# Hold-to-act: the GCS publishes this at >= 10 Hz while a button is
# held; the driver applies a 200 ms watchdog and zeros both velocities
# if commands stop arriving. The driver translates each non-NaN
# velocity into a moteus `make_position(position=NaN, velocity=...,
# maximum_torque=...)` per the test harness in
# src/astrotech_rover/scripts/auger_keys_test_harness.py.

float32 lead_screw_velocity_rev_s
float32 lead_screw_max_torque_nm
float32 auger_velocity_rev_s
float32 auger_max_torque_nm
