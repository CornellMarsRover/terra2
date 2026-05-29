# Live status of the analysis sequencer, published at status_rate_hz whether
# idle or running. The GUI subscribes to this for progress.
#
# The CAN-FD pump/servo boards give NO feedback (see astrotech_canfd
# API_NOTES.md), so this is commanded-only: it reflects which step the driver
# has issued and how long it has been running, not sensed completion. Sequences
# run to completion or hard-abort on cancel.

bool running                    # true while a sequence is executing
int8 active_sequence_id         # sequence currently running (0 when idle)
int32 current_step              # 1-based index of the running step (0 when idle)
int32 total_steps
string current_step_description
float32 elapsed_seconds

# Result of the most recently finished or cancelled sequence (valid once one
# has run; while running=true a new one is in progress).
bool last_success
string last_message
int32 last_completed_step
