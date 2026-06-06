---
status: in-progress
sprint: '003'
tickets:
- 003-001
---

# Plan: `velocity_chart.py` bench tool for SerialHubControl (RHSP)

## Context

We want a live telemetry/visualization tool for the RHSP library, modeled on
`/Volumes/Proj/proj/RobotProjects/radio-robot-c/tests/bench/velocity_chart.py`.
Its purpose is to **validate the two-layer motor control** being built in this
repo: the hub's onboard per-motor velocity PID (inner loop) plus the host-side
**ratio coordinator** (`RatioDrive`, the pending issue
`plan-two-layer-motor-velocity-control-on-hub-pid-host-side-ratio-coordinator`).

What we want to see on screen:
- **Two strip charts** (one per motor): measured velocity vs. time, converging
  quickly to the commanded setpoint (a dashed reference line).
- **One X/Y phase plot**: motor-A velocity vs. motor-B velocity. As one motor is
  loaded past its capability, the operating point should slide *down along a line
  whose slope is the commanded ratio* — the signature that the ratio coordinator
  is holding the ratio in actual velocities while de-rating the pack.

Required controls (carried over from the reference): **SPACE** starts/stops the
motors *and* the chart streaming; **Q** quits the program.

Per stakeholder direction, the script is written **against the locked `RatioDrive`
API** (from the control-plan issue) on the assumption that coordinator exists by
the time the script is finished — no "no-governor" fallback. Units are **raw
encoder counts/s**; default channels **0 & 1**; default commanded ratio **1.0**.

## Key facts established during exploration

- Connect: `rhsp.connect(port) -> Hub`; `rhsp.enumerate_hubs() -> list[str]`
  (REV serials start with "D"). Hub is a context manager: `with hub:` runs the
  keep-alive heartbeat (daemon thread) and fail-safes on exit. `hub.init_peripherals()`
  brings motors/servos up. See [discovery.py](src/rhsp/discovery.py), [hub.py](src/rhsp/hub.py).
- Measured velocity: `bulk = hub.bulk_input()` (one atomic transaction) →
  `bulk.motor0_velocity`, `bulk.motor1_velocity` (signed 16-bit counts/s), plus
  `bulk.monotonic_time_ms`. `Session` is RLock-guarded, so polling from a worker
  thread is safe alongside the keep-alive thread and `RatioDrive`'s loop.
  See [bulk.py](src/rhsp/devices/bulk.py), [motor.py](src/rhsp/devices/motor.py).
- **`RatioDrive` is not built yet** — its API is locked in the control-plan issue:
  `RatioDrive(hub, weights: Mapping[int,float], *, rate_hz=..., ...)`,
  `set_speed(counts_s)`, `set_ratio(ratio, pair=(0,1))`, `stop(brake=False)`,
  `start()/stop_loop()/close()`, and `__enter__/__exit__`. This is the single
  external dependency the script consumes; it lives in `src/rhsp/control.py`.
- **matplotlib/numpy are NOT installed** — the repo only depends on `pyserial`.
  They must be added as an optional dependency group.
- Existing runnable scripts live in [examples/](examples/) and are run with
  `uv run python examples/<name>.py`. Hardware notes (memory): real hub on
  `/dev/cu.usbserial-DQ3M375O`, fw 1.8.2; **actuators need the 12V battery**.

## Approach

### 1. New file: `examples/velocity_chart.py`

Adapt the reference script's proven structure (worker thread + queues + main-thread
matplotlib render loop), retargeted to RHSP. Name has no `test_` prefix so pytest
does not collect this interactive GUI tool (matches the reference's naming).

**CLI args** (`argparse`):
- `--port` (default: auto via `enumerate_hubs()[0]`)
- `--channels` (default `0,1`) — the two managed motor channels A,B
- `--ratio` (default `1.0`) — commanded B:A ratio; weights `{A:1.0, B:ratio}`
- `--speed` (default `1000`) — base scale in counts/s passed to `set_speed()`
- `--window` (default `8.0`) — rolling time window, seconds
- `--vmax` (default: `1.5*speed`, min-clamped) — symmetric axis limit for all plots
- `--rate` (default `50`) — chart sampling Hz (worker poll rate)

**Worker thread** (`_stream_worker`, one per SPACE-start; torn down on SPACE-stop) —
mirrors the reference's fresh-connection-per-start resilience:
```
with rhsp.connect(port) as hub:            # keep-alive heartbeat on
    hub.init_peripherals()
    with RatioDrive(hub, {a: 1.0, b: ratio}) as drive:   # locked API
        drive.set_speed(speed)
        while not stop_event.is_set():
            bulk = hub.bulk_input()                       # robust: read straight from hw
            t = time.monotonic()
            data_queue.put((t, getattr(bulk, f"motor{a}_velocity"),
                                getattr(bulk, f"motor{b}_velocity")))
            stop_event.wait(1.0 / rate)
    # with-blocks: drive.close() (zero+disable+restore) then hub fail-safe + disconnect
```
- Charting reads velocities **directly from `hub.bulk_input()`** (ground truth from
  the hardware), so the plots are decoupled from `RatioDrive`'s internal state —
  `RatioDrive` is used only to *drive*. The `RatioDrive` import is the one isolated
  integration point; if the shipped API differs slightly, only this block changes.
- Status strings pushed to a `status_queue` for the title (CONNECTING / RUNNING /
  ERROR / STOPPED), exactly like the reference.

**Main thread** (`main`): matplotlib figure with a 2×2 `GridSpec` — `ax1` (motor A
strip), `ax2` (motor B strip) in the left column, `ax3` (phase plot) spanning the
right column. Reuse the reference's `dark_background` style, deque buffers
(`maxlen = window * rate`), and the `plt.ion()` + manual draw/flush loop at ~30 fps.
- Strip charts: dashed yellow line at the commanded setpoint
  (`speed` for A, `round(speed*ratio)` for B); y-limits `[-vmax, vmax]`.
- Phase plot: `set_aspect("equal")`, limits `[-vmax,vmax]`; **reference line of slope
  = ratio** (`y = ratio * x`, i.e. commanded vB vs vA), grey history trace, red
  current-point dot. Units labeled "counts/s".
- macOS → `matplotlib.use("MacOSX")` (this host is darwin); Linux → `TkAgg`
  (carried over from the reference's thread-safe backend selection).

**Keyboard** (`fig.canvas.mpl_connect("key_press_event", ...)`):
- `" "` (SPACE): if a worker is alive → set its stop event (stop motors + stream);
  else → spawn a fresh stop-event + worker thread (start motors + stream).
- `"q"`: set a quit flag that breaks the render loop; the `finally` block stops any
  worker (`drive.close()` + hub fail-safe via the `with` exits) and closes figures.

### 2. Add the charting dependency to `pyproject.toml`

Add an optional group so the core library stays at just `pyserial`:
```toml
[project.optional-dependencies]
bench = ["matplotlib>=3.8", "numpy>=1.26"]
```
Run with: `uv run --extra bench python examples/velocity_chart.py`.

## Files

- **New:** [examples/velocity_chart.py](examples/velocity_chart.py) — the tool.
- **Edit:** [pyproject.toml](pyproject.toml) — add `bench` optional-dependencies.

## Dependency / sequencing note

The script imports `RatioDrive` from `src/rhsp/control.py`, which the pending
control-plan issue will create. The chart can be authored now against that locked
API, but **end-to-end hardware runs require `RatioDrive` to be implemented first**
(its own CLASI sprint). Until then the script imports will fail fast with a clear
message. This is example/tooling code (like the other `examples/`), so writing it
is a direct change — no new sprint artifact is needed for the chart itself.

## Verification

1. **Static / import check:** `uv run --extra bench python -c "import matplotlib, numpy"`
   and `python -m pyflakes examples/velocity_chart.py` (or `uv run python -c "import ast,
   pathlib; ast.parse(pathlib.Path('examples/velocity_chart.py').read_text())"`).
2. **Regression gate:** `uv run pytest` stays green (the new GUI script is not
   collected; pyproject edit must not break resolution).
3. **Live hardware** (needs the hub on `/dev/cu.usbserial-DQ3M375O` **and the 12V
   battery connected** for the motors to move, and `RatioDrive` implemented):
   `uv run --extra bench python examples/velocity_chart.py --speed 1000 --ratio 1.0`
   - Press **SPACE** → both motors spin up; both strip charts converge to the dashed
     setpoint; the phase dot tracks along the slope-=-ratio line.
   - Hand-load one motor → its strip chart droops and the phase point slides *down
     the ratio line* (ratio held in actual velocities), recovering on release.
   - Press **SPACE** again → motors stop and streaming pauses; **Q** → clean exit
     (motors fail-safed, window closed).
