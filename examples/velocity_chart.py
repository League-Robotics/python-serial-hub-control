"""Interactive live-telemetry bench tool for REV Hub motor velocity.

Three matplotlib panels: motor-A velocity strip chart (top-left), motor-B
velocity strip chart (bottom-left), and an X/Y phase plot (right column) of
vA vs vB.  Drive motors via RatioDrive.  Read measured velocities directly
from hub.bulk_input() (ground truth).

Controls:
  SPACE — start / stop motors and chart streaming
  Q     — quit and close window

Usage::

    uv run --extra bench python examples/velocity_chart.py [--port DEV] [--speed N] \\
        [--channels A,B] [--ratio R] [--window S] [--vmax V] [--rate HZ]

Axis scaling:
  Each strip chart and each phase-plot axis scales independently to fit its
  own visible data plus the commanded setpoint.  Pass ``--vmax`` to cap the
  auto-scaled half-span at a fixed value (useful for reproducible screenshots).
"""

from __future__ import annotations

import argparse
import platform
import queue
import threading
import time
from collections import deque
from typing import Any

# Backend must be selected BEFORE importing matplotlib.pyplot.
import matplotlib

matplotlib.use("MacOSX" if platform.system() == "Darwin" else "TkAgg")

import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.gridspec as gridspec  # noqa: E402
import numpy as np  # noqa: E402

import rhsp  # noqa: E402
from rhsp.control import RatioDrive  # noqa: E402


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="REV Hub motor velocity bench tool — live strip chart + phase plot.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--port",
        default=None,
        metavar="DEV",
        help="Serial port (e.g. /dev/cu.usbserial-DQ3M375O). "
             "Omit to auto-discover via enumerate_hubs().",
    )
    p.add_argument(
        "--channels",
        default="0,1",
        metavar="A,B",
        help="Two motor channel indices (comma-separated).",
    )
    p.add_argument(
        "--ratio",
        type=float,
        default=1.0,
        metavar="R",
        help="Velocity ratio for channel B relative to A (B weight = ratio).",
    )
    p.add_argument(
        "--speed",
        type=int,
        default=1000,
        metavar="N",
        help="Target speed for channel A in encoder counts/s.",
    )
    p.add_argument(
        "--window",
        type=float,
        default=8.0,
        metavar="S",
        help="Scrolling window width in seconds.",
    )
    p.add_argument(
        "--vmax",
        type=float,
        default=None,
        metavar="V",
        help=(
            "Cap on the auto-scaled y-axis half-span (counts/s). "
            "When supplied, this value acts as a ceiling: the auto-scaler grows "
            "axes to fit data/setpoints but never beyond --vmax. "
            "When omitted, axes scale freely to fit whatever data arrives. "
            "Precedence: explicit --vmax always wins over auto-scale."
        ),
    )
    p.add_argument(
        "--rate",
        type=int,
        default=50,
        metavar="HZ",
        help="Polling rate in Hz for the worker thread.",
    )
    p.add_argument(
        "--current-limit",
        type=int,
        default=2000,
        metavar="MA",
        dest="current_limit",
        help=(
            "Per-motor current limit in milliamps. De-rates the pack if any motor "
            "exceeds this threshold, preserving the commanded velocity ratio. "
            "Default 2000 mA (conservative — well below a typical 5 A bench supply trip). "
            "Set to 0 to disable current-aware de-rating."
        ),
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Axis-scaling helpers
# ---------------------------------------------------------------------------


def _fit_limit(
    values: "deque[int] | list[float]",
    setpoint: float,
    margin: float = 0.15,
    floor: float = 50.0,
    vmax_cap: float | None = None,
) -> float:
    """Return a symmetric half-span for a velocity axis.

    The result is ``max(max(|values|), |setpoint|, floor) * (1 + margin)``.
    When *vmax_cap* is not None it acts as a ceiling: the returned value is
    clamped to ``vmax_cap`` from above.  This is the ``--vmax`` override path.

    Parameters
    ----------
    values:    Iterable of measured velocities currently visible in the window.
               May be empty (e.g. before any data arrives).
    setpoint:  Commanded velocity for this channel (used as a minimum span).
    margin:    Fractional headroom added above the largest absolute value.
               Default 0.15 gives 15% headroom.
    floor:     Minimum half-span, preventing the axis from collapsing to zero
               when the motor is stopped.  Default 50 cnt/s.
    vmax_cap:  If not None, caps the returned value at this ceiling.
               Explicit ``--vmax`` passes this parameter; auto-scale omits it.
    """
    max_abs_data = max((abs(v) for v in values), default=0.0)
    raw = max(max_abs_data, abs(setpoint), floor) * (1.0 + margin)
    if vmax_cap is not None:
        raw = min(raw, float(vmax_cap))
    return raw


# ---------------------------------------------------------------------------
# Worker thread
# ---------------------------------------------------------------------------


def _stream_worker(
    port: str,
    ch_a: int,
    ch_b: int,
    ratio: float,
    speed: int,
    rate: int,
    current_limit_ma: int,
    data_queue: "queue.Queue[tuple[float, int, int, int, int]]",
    stop_event: threading.Event,
    status_queue: "queue.Queue[str]",
) -> None:
    """Worker thread: connect, drive, stream velocity+current samples, then clean up.

    Lifecycle messages pushed to *status_queue*:
      ``"CONNECTING"`` — before the serial open.
      ``"RUNNING"``    — after hub.init_peripherals() returns.
      ``"ERROR: ..."`` — on any exception.
      ``"STOPPED"``    — unconditionally in the finally block.

    Parameters
    ----------
    port:             Serial device path (already resolved — never None here).
    ch_a, ch_b:       Motor channel indices for the two strip charts.
    ratio:            Weight for channel B; channel A is always 1.0.
    speed:            Target velocity scale (counts/s).
    rate:             Poll frequency in Hz.
    current_limit_ma: Per-motor current limit passed to RatioDrive (mA).
                      Pass 0 to disable current-aware de-rating.
    data_queue:       Output queue of ``(monotonic_t, vA, vB, iA_mA, iB_mA)`` tuples.
                      ``iA_mA`` / ``iB_mA`` are the most-recently sampled motor
                      currents from RatioDrive.last_current_ma (0 when not yet
                      sampled or when current limiting is disabled).
    stop_event:       Set by the main thread to request shutdown.
    status_queue:     Status string queue read by the render loop.
    """
    status_queue.put("CONNECTING")
    try:
        with rhsp.connect(port) as hub:
            hub.init_peripherals()
            status_queue.put("RUNNING")
            # current_limit_ma=0 means "disabled"; pass None to RatioDrive in that case.
            # The default of 2000 mA is a conservative limit well below a typical
            # 5 A bench supply trip threshold.
            limit = current_limit_ma if current_limit_ma > 0 else None
            with RatioDrive(hub, {ch_a: 1.0, ch_b: ratio}, current_limit_ma=limit) as drive:
                drive.set_speed(speed)
                period = 1.0 / rate
                while not stop_event.wait(period):
                    bulk = hub.bulk_input()
                    t = time.monotonic()
                    vA = getattr(bulk, f"motor{ch_a}_velocity")
                    vB = getattr(bulk, f"motor{ch_b}_velocity")
                    # Current readings come from RatioDrive.last_current_ma (sampled
                    # every few ticks by the governor — no extra GetADC cost here).
                    cur = drive.last_current_ma
                    iA = cur.get(ch_a, 0)
                    iB = cur.get(ch_b, 0)
                    data_queue.put((t, vA, vB, iA, iB))
    except Exception as exc:
        status_queue.put(f"ERROR: {exc}")
    finally:
        status_queue.put("STOPPED")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:  # noqa: C901
    args = _parse_args()

    # --- Parse channels -------------------------------------------------------
    try:
        parts = args.channels.split(",")
        if len(parts) != 2:
            raise ValueError("need exactly two channel indices")
        ch_a, ch_b = int(parts[0].strip()), int(parts[1].strip())
    except ValueError as exc:
        print(f"Error: --channels must be two comma-separated integers: {exc}")
        return 1

    # --- Resolve vmax ---------------------------------------------------------
    # vmax_cap is None when the user did NOT pass --vmax (auto-scale freely).
    # When supplied, it caps the auto-scaled half-span from above.
    vmax_cap: float | None = args.vmax  # None → unconstrained auto-scale

    # --- Fail-fast: no hub and no port given ----------------------------------
    port: str | None = args.port
    if port is None:
        ports = rhsp.enumerate_hubs()
        if not ports:
            print(
                "Error: no REV Hub found and --port was not specified.\n"
                "Connect the hub via USB or pass --port /dev/cu.usbserial-XXXX."
            )
            return 2
        port = ports[0]

    # --- Build figure ---------------------------------------------------------
    with plt.style.context("dark_background"):
        fig = plt.figure(figsize=(12, 6))
        gs = gridspec.GridSpec(2, 2, figure=fig)

        ax_a = fig.add_subplot(gs[0, 0])
        ax_b = fig.add_subplot(gs[1, 0], sharex=ax_a)
        ax_phase = fig.add_subplot(gs[:, 1])

    fig.suptitle(
        "REV Hub motor velocity  [SPACE = connect]",
        color="white",
        fontsize=11,
    )

    # Compute initial axis half-spans from setpoints (no data yet).
    _sp_a = args.speed
    _sp_b = round(args.speed * args.ratio)
    _init_half_a = _fit_limit([], _sp_a, vmax_cap=vmax_cap)
    _init_half_b = _fit_limit([], _sp_b, vmax_cap=vmax_cap)

    # Strip chart — motor A
    ax_a.set_ylabel("velocity (counts/s)", color="white")
    ax_a.set_title(f"Motor {ch_a}", color="white", fontsize=9)
    ax_a.set_xlim(0, args.window)
    ax_a.set_ylim(-_init_half_a, _init_half_a)
    ax_a.axhline(0, color="grey", linewidth=0.5)
    ax_a.axhline(_sp_a, color="yellow", linewidth=1.0, linestyle="--", label=f"SP {_sp_a}")
    (line_a,) = ax_a.plot([], [], color="cyan", linewidth=1.2)
    ax_a.tick_params(labelbottom=False)

    # Strip chart — motor B
    ax_b.set_xlabel("time (s)", color="white")
    ax_b.set_ylabel("velocity (counts/s)", color="white")
    ax_b.set_title(f"Motor {ch_b}", color="white", fontsize=9)
    ax_b.set_xlim(0, args.window)
    ax_b.set_ylim(-_init_half_b, _init_half_b)
    ax_b.axhline(0, color="grey", linewidth=0.5)
    ax_b.axhline(_sp_b, color="yellow", linewidth=1.0, linestyle="--", label=f"SP {_sp_b}")
    (line_b,) = ax_b.plot([], [], color="lime", linewidth=1.2)

    # Phase plot — independent x/y limits, no set_aspect("equal").
    # Initial limits derive from each motor's setpoint independently.
    ax_phase.set_xlim(-_init_half_a, _init_half_a)
    ax_phase.set_ylim(-_init_half_b, _init_half_b)
    ax_phase.set_xlabel(f"Motor {ch_a} (counts/s)", color="white")
    ax_phase.set_ylabel(f"Motor {ch_b} (counts/s)", color="white")
    ax_phase.set_title("Phase  vB vs vA", color="white", fontsize=9)
    ax_phase.axhline(0, color="grey", linewidth=0.5)
    ax_phase.axvline(0, color="grey", linewidth=0.5)
    # Reference line y = ratio * x — data updated each frame to span visible xlim.
    (ref_line,) = ax_phase.plot(
        [-_init_half_a, _init_half_a],
        [-_init_half_a * args.ratio, _init_half_a * args.ratio],
        color="grey", linewidth=1.0, linestyle=":", label=f"y={args.ratio:.2f}x",
    )
    (phase_trace,) = ax_phase.plot([], [], color="dimgrey", linewidth=0.8, alpha=0.7)
    (phase_dot,) = ax_phase.plot([], [], "ro", markersize=6)

    plt.tight_layout()

    # --- Deque buffers --------------------------------------------------------
    maxlen = int(args.window * args.rate)
    buf_t: deque[float] = deque(maxlen=maxlen)
    buf_vA: deque[int] = deque(maxlen=maxlen)
    buf_vB: deque[int] = deque(maxlen=maxlen)

    # --- Hysteresis state for auto-scaling ------------------------------------
    # Track the current half-span for each axis.  On each frame we only update
    # the limit when the desired value differs by more than 5% from the current
    # value — this prevents distracting axis "breathing" when data is steady.
    half_a: list[float] = [_init_half_a]   # strip-chart motor A
    half_b: list[float] = [_init_half_b]   # strip-chart motor B
    half_px: list[float] = [_init_half_a]  # phase-plot x axis (motor A)
    half_py: list[float] = [_init_half_b]  # phase-plot y axis (motor B)

    _HYSTERESIS = 0.05  # only update when desired differs by more than 5%

    # --- Shared queues and worker state ---------------------------------------
    data_queue: queue.Queue[tuple[float, int, int, int, int]] = queue.Queue()
    status_queue: queue.Queue[str] = queue.Queue()
    worker_state: dict[str, Any] = {"thread": None, "stop": None}
    quit_flag: list[bool] = [False]
    # Latest current readings (mA) — updated each render frame from data_queue.
    last_current: dict[str, int] = {"A": 0, "B": 0}

    # --- Key handler ----------------------------------------------------------
    def _on_key(event: Any) -> None:
        if event.key == " ":
            thread = worker_state["thread"]
            if thread is not None and thread.is_alive():
                # Stop the running worker.
                worker_state["stop"].set()
                fig.suptitle(
                    "REV Hub motor velocity  [stopping…]",
                    color="white",
                    fontsize=11,
                )
            else:
                # Start a fresh worker.
                stop_evt = threading.Event()
                worker_state["stop"] = stop_evt
                t = threading.Thread(
                    target=_stream_worker,
                    args=(
                        port, ch_a, ch_b, args.ratio, args.speed,
                        args.rate, args.current_limit,
                        data_queue, stop_evt, status_queue,
                    ),
                    daemon=True,
                    name="velocity-chart-worker",
                )
                worker_state["thread"] = t
                t.start()
                fig.suptitle(
                    "REV Hub motor velocity  [CONNECTING…]",
                    color="white",
                    fontsize=11,
                )
        elif event.key == "q":
            quit_flag[0] = True

    fig.canvas.mpl_connect("key_press_event", _on_key)

    # --- Update function ------------------------------------------------------
    def _update() -> None:
        # Drain status queue.
        try:
            while True:
                msg = status_queue.get_nowait()
                if msg.startswith("ERROR:"):
                    fig.suptitle(
                        f"REV Hub motor velocity  [{msg}]",
                        color="red",
                        fontsize=11,
                    )
                elif msg == "RUNNING":
                    fig.suptitle(
                        "REV Hub motor velocity  [RUNNING — SPACE to stop]",
                        color="white",
                        fontsize=11,
                    )
                elif msg == "STOPPED":
                    fig.suptitle(
                        "REV Hub motor velocity  [STOPPED — SPACE to restart]",
                        color="white",
                        fontsize=11,
                    )
                elif msg == "CONNECTING":
                    fig.suptitle(
                        "REV Hub motor velocity  [CONNECTING…]",
                        color="white",
                        fontsize=11,
                    )
        except queue.Empty:
            pass

        # Drain data queue.
        try:
            while True:
                t, vA, vB, iA, iB = data_queue.get_nowait()
                buf_t.append(t)
                buf_vA.append(vA)
                buf_vB.append(vB)
                last_current["A"] = iA
                last_current["B"] = iB
        except queue.Empty:
            pass

        if not buf_t:
            return

        t_arr = np.array(buf_t)
        vA_arr = np.array(buf_vA)
        vB_arr = np.array(buf_vB)

        # Relative time clipped to [0, window].
        t_end = t_arr[-1]
        t_rel = np.clip(t_arr - (t_end - args.window), 0.0, args.window)

        line_a.set_data(t_rel, vA_arr)
        line_b.set_data(t_rel, vB_arr)
        phase_trace.set_data(vA_arr, vB_arr)
        phase_dot.set_data([vA_arr[-1]], [vB_arr[-1]])

        # --- Auto-scale strip charts (each axis independently) ----------------
        # Desired half-span for motor A: fit visible data + setpoint + margin.
        desired_a = _fit_limit(buf_vA, _sp_a, vmax_cap=vmax_cap)
        if abs(desired_a - half_a[0]) / half_a[0] > _HYSTERESIS:
            half_a[0] = desired_a
            ax_a.set_ylim(-half_a[0], half_a[0])

        # Desired half-span for motor B: independent — tracks speed*ratio.
        desired_b = _fit_limit(buf_vB, _sp_b, vmax_cap=vmax_cap)
        if abs(desired_b - half_b[0]) / half_b[0] > _HYSTERESIS:
            half_b[0] = desired_b
            ax_b.set_ylim(-half_b[0], half_b[0])

        # --- Auto-scale phase plot (x from motor A, y from motor B) -----------
        desired_px = _fit_limit(buf_vA, _sp_a, vmax_cap=vmax_cap)
        if abs(desired_px - half_px[0]) / half_px[0] > _HYSTERESIS:
            half_px[0] = desired_px
            ax_phase.set_xlim(-half_px[0], half_px[0])

        desired_py = _fit_limit(buf_vB, _sp_b, vmax_cap=vmax_cap)
        if abs(desired_py - half_py[0]) / half_py[0] > _HYSTERESIS:
            half_py[0] = desired_py
            ax_phase.set_ylim(-half_py[0], half_py[0])

        # Keep reference line y = ratio*x spanning the full visible x range.
        rx0, rx1 = ax_phase.get_xlim()
        ref_line.set_xdata([rx0, rx1])
        ref_line.set_ydata([args.ratio * rx0, args.ratio * rx1])

        # Update motor subplot titles with current readout when current limiting
        # is active (current_limit > 0).  Values come from RatioDrive.last_current_ma
        # via the data queue; they are 0 when not yet sampled.
        if args.current_limit > 0:
            iA = last_current["A"]
            iB = last_current["B"]
            lim = args.current_limit
            ax_a.set_title(
                f"Motor {ch_a}  I={iA} mA / {lim} mA", color="white", fontsize=9
            )
            ax_b.set_title(
                f"Motor {ch_b}  I={iB} mA / {lim} mA", color="white", fontsize=9
            )

    # --- Render loop ----------------------------------------------------------
    plt.ion()
    plt.show(block=False)

    try:
        while plt.fignum_exists(fig.number) and not quit_flag[0]:
            _update()
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.033)
    finally:
        # Stop any running worker cleanly.
        stop_evt = worker_state.get("stop")
        if stop_evt is not None:
            stop_evt.set()
        thread = worker_state.get("thread")
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
        plt.close("all")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
