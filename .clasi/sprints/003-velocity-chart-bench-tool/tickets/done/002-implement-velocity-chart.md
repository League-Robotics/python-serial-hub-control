---
id: 003-002
title: Implement examples/velocity_chart.py
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- 003-001
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
---

# 003-002: Implement examples/velocity_chart.py

## Description

Create `examples/velocity_chart.py` — the interactive live-telemetry bench tool
for the RHSP library. The script renders three matplotlib panels (two motor
velocity strip charts + one X/Y phase plot), drives motors via `RatioDrive`, and
reads measured velocities directly from `hub.bulk_input()`. It follows the
proven structure from the reference
`radio-robot-c/tests/bench/velocity_chart.py`: worker thread + queues +
main-thread matplotlib render loop, with fresh-connection-per-SPACE.

No library source files are modified. This ticket adds one new file to `examples/`.

## Acceptance Criteria

- [x] File `examples/velocity_chart.py` exists, has no `test_` prefix, and is
      not collected by `uv run pytest`.
- [x] AST parse succeeds: `uv run python -c "import ast, pathlib; ast.parse(pathlib.Path('examples/velocity_chart.py').read_text())"` exits 0.
- [ ] Running `uv run --extra bench python examples/velocity_chart.py` (no hub)
      opens a matplotlib window with three panels and title "REV Hub motor
      velocity  [SPACE = connect]".
- [ ] Pressing Q with the window focused closes the window cleanly (exit code 0).
- [x] CLI args `--port`, `--channels`, `--ratio`, `--speed`, `--window`,
      `--vmax`, `--rate` are accepted (`--help` lists them).
- [x] `uv run pytest` stays green (no regressions).

Hardware acceptance (requires `/dev/cu.usbserial-DQ3M375O` + 12V battery):
- [ ] SPACE starts motors; strip charts fill with live velocity traces converging
      to the dashed setpoint within ~5 s at `--speed 1000`.
- [ ] Phase dot tracks along the slope-ratio reference line.
- [ ] Second SPACE stops motors cleanly; title reverts to STOPPED state.
- [ ] Q exits from any state; window closes, motors fail-safed.

## Implementation Plan

### Approach

Adapt the reference script's structure to RHSP. Key structural differences from
the reference:
- Uses `rhsp.connect(port)` / `rhsp.enumerate_hubs()` instead of robot-radio
  serial primitives.
- Drives motors via `RatioDrive(hub, {a: 1.0, b: ratio})` context manager
  instead of custom protocol commands.
- Reads measured velocities via `hub.bulk_input()` directly (not from
  `RatioDrive.measured`).
- Three panels instead of two: strip-A (top-left), strip-B (bottom-left),
  phase plot (right column spanning both rows).
- Phase plot reference line is `y = ratio * x` (not `y = x`).

### File to Create

**`examples/velocity_chart.py`** — top-level structure:

```
_parse_args() -> argparse.Namespace
    --port, --channels (default "0,1"), --ratio (default 1.0),
    --speed (default 1000), --window (default 8.0),
    --vmax (default None, computed as max(1.5*speed, 100)),
    --rate (default 50)

_stream_worker(port, channels, ratio, speed, rate, data_queue, stop_event, status_queue)
    with rhsp.connect(port) as hub:
        hub.init_peripherals()
        with RatioDrive(hub, {a: 1.0, b: ratio}) as drive:
            drive.set_speed(speed)
            while not stop_event.wait(1.0/rate):
                bulk = hub.bulk_input()
                t = time.monotonic()
                vA = getattr(bulk, f"motor{a}_velocity")
                vB = getattr(bulk, f"motor{b}_velocity")
                data_queue.put((t, vA, vB))
    status_queue sends: "CONNECTING", "RUNNING", "ERROR: ...", "STOPPED"

main() -> int
    parse args
    select matplotlib backend (MacOSX / TkAgg)
    build figure: GridSpec(2,2), ax1=gs[0,0], ax2=gs[1,0], ax3=gs[:,1]
    configure axes: dark_background, ylim=[-vmax,vmax], dashed setpoint lines
    phase plot: set_aspect("equal"), xlim/ylim=[-vmax,vmax],
                reference line y=ratio*x
    deque buffers: maxlen = int(window * rate)
    worker_state = {"thread": None, "stop": None}
    mpl_connect("key_press_event", _on_key)
    plt.ion(); plt.show(block=False)
    render loop: while plt.fignum_exists(fig.number): _update(); draw/flush; sleep(0.033)
    finally: stop worker, plt.close("all")
```

**Worker thread detail**: Sends `"CONNECTING"` before the connect call, `"RUNNING"`
after `hub.init_peripherals()` completes, `"ERROR: <msg>"` on any exception, and
`"STOPPED"` in `finally`. The `with` blocks handle teardown on all exit paths
(normal return, exception, or `stop_event` set by SPACE-stop).

**Main thread _update()**: Drain `status_queue` → update title text. Drain
`data_queue` → append to deques. If deques non-empty: build numpy arrays,
compute `rel = clip(t_arr - (t_arr[-1] - window), 0, window)`, call
`set_data()` on all five artists (line_A, line_B, phase_trace, phase_dot,
optionally update phase ref line if ratio changes at runtime — not required for
v1). Return artist tuple.

**_on_key(event)**:
- `" "` (space): if worker alive → set stop_event; else → fresh stop_event +
  fresh thread, start thread, update title.
- `"q"`: set quit flag that breaks the `while plt.fignum_exists(...)` loop.

**Backend selection** (before `import matplotlib.pyplot`):
```python
import matplotlib, platform
matplotlib.use("MacOSX" if platform.system() == "Darwin" else "TkAgg")
```

### Files to Create

- `examples/velocity_chart.py`

### Files to Modify

None (no library files touched).

### Testing Plan

**Hardware-free**:
1. `uv run python -c "import ast, pathlib; ast.parse(pathlib.Path('examples/velocity_chart.py').read_text())"` — syntax check.
2. `uv run --extra bench python examples/velocity_chart.py --help` — confirms argparse wired up, no import errors.
3. `uv run pytest` — confirms no regressions and script not collected.

**Hardware** (manual, `/dev/cu.usbserial-DQ3M375O` + 12V battery):
- Run `uv run --extra bench python examples/velocity_chart.py --speed 1000 --ratio 1.0`.
- SPACE: motors spin, strip charts converge, phase dot tracks ratio line.
- Hand-load motor A: strip droops, phase dot slides along ratio line.
- Second SPACE: motors stop cleanly.
- Q: clean exit.

### Documentation Updates

No doc updates required. The script has a module docstring and `--help` output
that serve as usage documentation.
