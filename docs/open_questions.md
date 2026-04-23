# Open Questions — Phase 1

Most of the earlier questions don't actually block Phase 1 and aren't worth
chasing right now; the rover-side reality check will settle them faster than
a document exchange will. What's left:

## Actually blocking (answer before competition, not before merge)

1. **URC 2026 RF bandwidth cap.**
   `docs/bandwidth_audit.md` uses a 5 Mbps working assumption based on prior
   years. Confirm the 2026 rulebook before we freeze the Foxglove topic
   whitelist.

## Ask the Astrotech team (Slack, low-priority)

2. **What do the two analysis sequences actually do, how many sites are
   there, and what do the five mixing-servo positions correspond to
   physically?**
   We need this to label the operator buttons correctly. The team hasn't
   written software yet so this is a hardware/mission-design question, not
   a code question. Draft Slack message is in the "Slack drafts" section
   below. Not blocking — can be debugged on the rover later if nobody has
   answers today.

## Decisions we made / assumptions we're baking in

- **Auger + astrotech BDC motors: moteus** (confirmed by team).
  The GCS publishes velocity on a `/astrotech/auger/*/cmd_velocity` topic
  and the driver wraps `send_moteus_command_sync` from
  `cmr_rovernet/rovernet_utils.py`.
- **Mixing servo controller family: TBD, but the GCS contract is
  controller-agnostic** — panel calls `cmr_msgs/srv/MixingServoPreset`,
  whoever writes the driver matches the service. Not worth pre-answering.
- **Cameras stay on `foxglove_msgs/CompressedVideo` (H.264)** rather than
  moving to `sensor_msgs/CompressedImage`. Lower bandwidth, driver already
  exists in `usb_camera_publisher/publisher.py`.
- **`foxglove_bridge` ships as a plain `launch_ros.Node`**, not a
  Fabric-managed node. Simpler; revisit if we need fault recovery on the
  bridge itself.

## Deferred until rover bring-up (will be debugged on-hardware)

- Which `/dev/videoN` is auger / site / analysis cam. Resolve with a YAML in
  `src/cmr_cams/config/` when cameras are plugged in.
- Snapshot naming / storage convention (only matters when we add the
  feature).
- Whether external packages referenced in `cmr_cams/config/*.toml`
  (`cmr_cv`, `cmr_arm`, `cmr_control`, `cmr_demo`) are actually needed on
  the rover, or are stale config pointing at retired code paths. Likely the
  latter given the dead-code situation — confirm on bring-up.
- Whether per-camera bitrate / framerate should be exposed as ROS params
  for live tuning over a degraded RF link. Nice-to-have, not required.

## Slack drafts

### To #astrotech

> Hey — starting on the GCS (ground station) side for science payload and
> trying to sketch out what buttons the operator will have. Couple of
> questions, nothing urgent:
>
> 1. For the two "analysis sequences" — roughly, what happens during each
>    one? Is it the same motions run on two different samples / sites, or
>    are they actually different sequences of motions? (E.g. Seq 1 = dig +
>    mix + Raman on site A, Seq 2 = same thing on site B vs. Seq 1 and
>    Seq 2 doing different science steps.)
> 2. How many physical sample sites / bays does the payload have?
> 3. The mixing servo has five positions in the task brief (S1, S2, CO2_1,
>    CO2_2, Retract) — can you describe what each one does physically so
>    we label the buttons right?
> 4. Anything else the operator should be able to trigger from the ground
>    station that we might not know about (heaters, lids, valves, etc.)?
>
> Totally fine if some of this is still TBD — we'll figure it out on the
> rover. Just want to make sure we're building buttons for what the
> payload actually does, not what I'm guessing it does.
