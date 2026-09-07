# Open issues

Known, unresolved issues. Each states the issue as observed, without root-cause
analysis.

## Cushion depth is not sized from measured render cost

The per-head demand cushion is sized by a frame-size heuristic
(`StretchTables::fillTarget() = activeW / 2`), not from the measured worst-case
render cost or the concurrent-render count.

Observed on the bench (`make PROFILE=1`, HLTH line) prior to the frame-
proportional change:

- Rotating to the largest FFT frame size (16384) picks up underruns (`du` in the
  low hundreds and climbing) when duration is longer. `max_us` at 16384 was
  ~18000 with a cushion of ~11 ms (fixed 1024/512).
- With ring-out remnants active at frame size 4096, `du` climbs (into the
  thousands) as `rmn` reaches 3-4, though `max_us` stays ~3600.
- Moving to smaller frames runs clean; moving back to larger frames reintroduces
  the underruns. Voice count is unchanged across the transition.
- One HLTH line during a starved second reported `svc_us=4274099524` /
  `max_us=4273496017` (a 32-bit wrap), alongside a `du` spike.

Open questions:

- What `fillTarget` value is actually required as a function of active frame size
  and the concurrent-render cap (`SS_RENDER_CAP`), confirmed against `max_us` /
  `fmin` / `du` at 16384 on hardware.
- Whether `SS_RENDER_CAP = 6` is sustainable at 16384.
- The 32-bit wrap in the profiler's `svc_us` / `max_us` during a starved second.
