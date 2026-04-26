# Honest, commanded-only telemetry. The Astrotech CAN-FD library exposes no
# motor feedback (no encoder, current, or completion ack), so anything beyond
# "what we last commanded and when" would be fabricated.
# See docs/astrotech_canfd_library_notes.md §5.

uint8 COMMAND_KIND_NONE = 0
uint8 COMMAND_KIND_UP = 1
uint8 COMMAND_KIND_DOWN = 2
uint8 COMMAND_KIND_SPIN_FORWARD = 3
uint8 COMMAND_KIND_SPIN_BACKWARD = 4

uint8 last_command_kind
float32 commanded_duration
builtin_interfaces/Time commanded_at
