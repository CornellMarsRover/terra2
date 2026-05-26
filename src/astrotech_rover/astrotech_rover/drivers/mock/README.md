# Mock drivers (simulation only)

These were the **simulation** drivers used to build and test the Astrotech
Foxglove GUI before the real hardware drivers existed. They publish
plausible fake telemetry / accept commands and echo state, so the panels
can be developed and demoed with **no rover hardware attached**.

**They are no longer the default.** `astrotech_node` runs the *real*
drivers (in the parent `drivers/` folder) by default. The mock drivers
here are opt-in, one feature at a time, via env vars:

```
URC_AUGER_MOCK=1          # fake auger instead of the moteus stack
URC_MIXING_SERVO_MOCK=1   # fake mixing servo instead of the CMR servo board
URC_RAMAN_MOCK=1          # synthetic spectrum instead of the TCD1340
URC_ENV_MOCK=1            # synthetic CO2/temp/humidity instead of the SCD-30
```

`analysis_sequencer.py` has no real driver yet, so the mock sequencer is
still always used for the analysis panel (Phase 2b will add the real one).

Keep these around for GUI development, demos, and CI — they're handy when
no rover is in front of you. Delete only if that workflow goes away.
