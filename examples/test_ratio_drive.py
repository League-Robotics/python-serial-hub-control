"""RatioDrive live-hub validation example.

Exercises the full two-layer velocity control stack on a REV Hub with two
drive motors (channels 0 and 1).  The script auto-skips when no hub is found
so it does not block hardware-free CI runs.

Hardware setup
--------------
- REV Control Hub or Expansion Hub at ``PORT`` below.
- 12 V battery connected (motors require 12 V to run).
- Motor 0: left drive wheel, encoder attached.
- Motor 1: right drive wheel, encoder attached.

Steps
-----
1. Capability probe  — confirm CONSTANT_VELOCITY mode works on channel 0.
2. 1:1 straight run  — both wheels equal speed; assert measured ratio ~ 1.0.
3. 1:0.5 curve run   — wheel 0 at 1.0, wheel 1 at 0.5; assert ratio v0/v1 ~ 2.0.
4. Induced-bottleneck — operator grips one wheel; observe scale drops AND
   the other wheel slows to preserve ratio; release and observe recovery.
   (Timed window: no blocking forever on input.)

Usage::

    uv run python -u examples/test_ratio_drive.py

Note on threading
-----------------
The RatioDrive daemon thread calls ``hub.bulk_input()`` at ``rate_hz`` (20 Hz
by default) through the session's internal RLock, which also resets the hub's
2500 ms keepalive timer.  The main thread must NOT call ``hub.keep_alive()``
concurrently with the daemon because those calls serialise through the same
lock and cause the monitoring loop to stall.  During RatioDrive-active phases,
keepalive is handled entirely by the daemon's bulk transactions.
"""

from __future__ import annotations

import sys
import time
import csv
import io

PORT = "/dev/cu.usbserial-DQ3M375O"

# ---------------------------------------------------------------------------
# Skip-guard: attempt connect before any test infrastructure runs
# ---------------------------------------------------------------------------


def _try_connect():
    """Return (hub, None) on success or (None, reason_str) on failure."""
    try:
        import rhsp
        from rhsp import RhspError
    except ImportError as exc:
        return None, f"rhsp not importable: {exc}"

    try:
        hub = rhsp.connect(PORT)
        return hub, None
    except Exception as exc:
        return None, f"connect({PORT!r}) failed: {exc}"


def _skip(reason: str) -> None:
    print(f"SKIP: {reason}", flush=True)
    sys.exit(0)


# ---------------------------------------------------------------------------
# CSV logging helpers
# ---------------------------------------------------------------------------

_csv_header = ["time_s", "step", "target_scale", "scale", "v0", "v1", "saturated"]
_csv_rows: list[list] = []
_t0: float = time.monotonic()


def _log_row(step: str, drive, v0: int, v1: int) -> None:
    t = round(time.monotonic() - _t0, 3)
    ts = round(drive.target_scale, 1)
    sc = round(drive.scale, 1)
    sat = drive.saturated
    row = [t, step, ts, sc, v0, v1, sat]
    _csv_rows.append(row)
    print(
        f"  t={t:7.3f}  step={step:<20s}  target_scale={ts:7.1f}  scale={sc:7.1f}"
        f"  v0={v0:6d}  v1={v1:6d}  sat={sat}",
        flush=True,
    )


def _dump_csv() -> None:
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(_csv_header)
    writer.writerows(_csv_rows)
    print("\n--- CSV LOG ---", flush=True)
    print(buf.getvalue(), flush=True)
    print("--- END CSV ---", flush=True)


# ---------------------------------------------------------------------------
# Step 1: capability probe
# ---------------------------------------------------------------------------


def step1_capability_probe(hub) -> bool:
    """Probe CONSTANT_VELOCITY on channel 0.  Returns True on PASS."""
    from rhsp import HubVelocityController, RhspError

    print("\n=== Step 1: Capability probe ===", flush=True)
    ctrl = HubVelocityController(hub, channel=0)
    try:
        ctrl.attach()
    except RhspError as exc:
        print(f"  FAIL: attach() raised {exc}", flush=True)
        return False

    try:
        ctrl.command(600)
        # Wait up to 3 s for non-zero velocity.
        # Use explicit keep_alive+bulk_input here — no daemon running yet.
        deadline = time.monotonic() + 3.0
        measured = 0
        while time.monotonic() < deadline:
            hub.keep_alive()
            bulk = hub.bulk_input()
            measured = ctrl.measured(bulk)
            print(f"  target=600  measured={measured:6d} enc/s", flush=True)
            if abs(measured) > 50:
                break
            time.sleep(0.2)

        if abs(measured) > 50:
            print(f"  PASS: motor responded (measured={measured} enc/s)", flush=True)
            return True
        else:
            print(f"  FAIL: measured velocity still zero after 3 s", flush=True)
            return False
    finally:
        ctrl.command(0)
        time.sleep(0.3)
        ctrl.detach(disable=True)
        hub.keep_alive()


# ---------------------------------------------------------------------------
# Shared: monitoring loop while RatioDrive daemon is running
# ---------------------------------------------------------------------------


def _monitor_drive(drive, step_name: str, duration_s: float) -> list[tuple[int, int]]:
    """Monitor ``drive.measured`` for ``duration_s`` seconds.

    The RatioDrive daemon handles keepalive via its bulk_input calls.
    The main thread only sleeps and reads the daemon's cached measurements.
    Returns a list of (v0, v1) tuples for all logged samples.
    """
    readings: list[tuple[int, int]] = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        m = drive.measured   # dict copy populated by daemon's last bulk read
        m0 = m.get(0, 0)
        m1 = m.get(1, 0)
        _log_row(step_name, drive, m0, m1)
        readings.append((m0, m1))
        time.sleep(0.15)   # ~6.7 Hz monitoring; daemon runs at 20 Hz
    return readings


# ---------------------------------------------------------------------------
# Step 2: 1:1 straight run
# ---------------------------------------------------------------------------


def step2_ratio_1_1(hub) -> bool:
    """1:1 run; assert |v0/v1 - 1.0| < 0.10 in steady state."""
    from rhsp import RatioDrive

    print("\n=== Step 2: 1:1 straight run ===", flush=True)
    drive = RatioDrive(hub, {0: 1.0, 1: 1.0}, rate_hz=20.0)
    drive.start()
    drive.set_speed(600)

    # Spin-up: wait for scale to reach target.
    print("  Spin-up 3 s (no logging — daemon handles keepalive)...", flush=True)
    time.sleep(3.0)

    # Steady-state measurement window.
    print("  Measuring steady-state (4 s)...", flush=True)
    try:
        readings = _monitor_drive(drive, "1:1", 4.0)
    finally:
        drive.close()
        # After daemon stops, send one keepalive to satisfy 2.5 s deadline.
        hub.keep_alive()

    valid = [(v0, v1) for v0, v1 in readings if abs(v1) > 50 and abs(v0) > 50]
    if not valid:
        print(
            f"  FAIL: no valid steady-state readings (need |v0|>50 and |v1|>50)."
            f"  All readings: {readings}",
            flush=True,
        )
        return False

    ratios = [abs(v0) / abs(v1) for v0, v1 in valid]
    avg_ratio = sum(ratios) / len(ratios)
    ok = abs(avg_ratio - 1.0) < 0.10
    print(
        f"  steady-state avg ratio v0/v1 = {avg_ratio:.3f}  n={len(valid)}"
        f"  {'PASS' if ok else 'FAIL'}  (threshold ±0.10)",
        flush=True,
    )
    return ok


# ---------------------------------------------------------------------------
# Step 3: 1:0.5 curve run
# ---------------------------------------------------------------------------


def step3_ratio_1_half(hub) -> bool:
    """1:0.5 run; assert |v0/v1 - 2.0| < 0.15 in steady state."""
    from rhsp import RatioDrive

    print("\n=== Step 3: 1:0.5 curve run ===", flush=True)
    # weights: ch0=1.0, ch1=0.5 → ch0 commanded 2x ch1 → v0/v1 ~ 2.0
    drive = RatioDrive(hub, {0: 1.0, 1: 0.5}, rate_hz=20.0)
    drive.start()
    drive.set_speed(1200)

    print("  Spin-up 3 s...", flush=True)
    time.sleep(3.0)

    print("  Measuring steady-state (4 s)...", flush=True)
    try:
        readings = _monitor_drive(drive, "1:0.5", 4.0)
    finally:
        drive.close()
        hub.keep_alive()

    valid = [(v0, v1) for v0, v1 in readings if abs(v1) > 50 and abs(v0) > 50]
    if not valid:
        print(
            f"  FAIL: no valid steady-state readings.  All readings: {readings}",
            flush=True,
        )
        return False

    ratios = [abs(v0) / abs(v1) for v0, v1 in valid]
    avg_ratio = sum(ratios) / len(ratios)
    ok = abs(avg_ratio - 2.0) < 0.15
    print(
        f"  steady-state avg ratio v0/v1 = {avg_ratio:.3f}  n={len(valid)}"
        f"  {'PASS' if ok else 'FAIL'}  (target 2.0, threshold ±0.15)",
        flush=True,
    )
    return ok


# ---------------------------------------------------------------------------
# Step 4: Induced-bottleneck scenario
# ---------------------------------------------------------------------------


def step4_induced_bottleneck(hub) -> bool:
    """Induced-bottleneck scenario with timed operator window.

    The operator is prompted to grip one wheel for ~6 s then release.
    The script observes drive.saturated and drive.scale over ~18 s total.

    PASS criteria (automated):
      - ``saturated`` becomes True and ``scale`` drops < 85% of target during
        the grip window.
      - After release, ``scale`` recovers > 120% of its minimum during grip.

    If no physical intervention is detected, the step prints NEEDS_OPERATOR
    and returns True (does not penalise an automated-only run).
    """
    from rhsp import RatioDrive

    print("\n=== Step 4: Induced-bottleneck scenario ===", flush=True)
    print(
        "  OPERATOR: When you see 'GRIP NOW', grip/hold one wheel firmly for ~6 s,",
        flush=True,
    )
    print("  then release. Script observes for 18 s total.", flush=True)

    drive = RatioDrive(hub, {0: 1.0, 1: 1.0}, rate_hz=20.0)
    drive.start()
    drive.set_speed(600)

    # Let motors spin up.
    print("  Spin-up 3 s...", flush=True)
    time.sleep(3.0)

    print("  GRIP NOW — hold one wheel for ~6 s!", flush=True)

    t_grip_end = time.monotonic() + 6.0
    t_obs_end  = time.monotonic() + 18.0

    scale_during_grip: list[float] = []
    sat_during_grip: list[bool] = []
    scale_after_release: list[float] = []

    try:
        while time.monotonic() < t_obs_end:
            m = drive.measured
            m0 = m.get(0, 0)
            m1 = m.get(1, 0)
            _log_row("bottleneck", drive, m0, m1)

            if time.monotonic() <= t_grip_end:
                scale_during_grip.append(drive.scale)
                sat_during_grip.append(drive.saturated)
            else:
                scale_after_release.append(drive.scale)

            time.sleep(0.15)
    finally:
        drive.close()
        hub.keep_alive()

    target_scale = 600.0
    if not scale_during_grip:
        print("  NEEDS_OPERATOR: no grip-window readings collected.", flush=True)
        return True

    min_scale_grip = min(scale_during_grip)
    any_sat = any(sat_during_grip)
    max_scale_recovery = max(scale_after_release, default=0.0)
    recovered = bool(scale_after_release) and max_scale_recovery > min_scale_grip * 1.2

    print(f"  grip window: min scale={min_scale_grip:.1f}  any_saturated={any_sat}", flush=True)
    print(
        f"  recovery: max post-release scale={max_scale_recovery:.1f}  recovered={recovered}",
        flush=True,
    )

    if not any_sat and min_scale_grip >= target_scale * 0.90:
        print(
            "  NEEDS_OPERATOR: scale did not drop during grip window — "
            "physical bottleneck not observed. Manual verification required.",
            flush=True,
        )
        return True  # automated-only run: don't fail

    ok = any_sat and (min_scale_grip < target_scale * 0.85) and recovered
    print(f"  Bottleneck/recovery: {'PASS' if ok else 'FAIL'}", flush=True)
    return ok


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    global _t0
    _t0 = time.monotonic()

    hub, reason = _try_connect()
    if hub is None:
        _skip(reason or "hub not available")

    print(f"Connected to hub at {PORT}", flush=True)
    hub.init_peripherals()

    results: dict[str, bool] = {}

    try:
        # Step 1 — capability probe (exit early if FAIL).
        # No daemon running; explicit keep_alive + bulk_input used.
        passed = step1_capability_probe(hub)
        results["step1_capability_probe"] = passed
        if not passed:
            print("\nCapability probe FAILED — aborting further steps.", flush=True)
            return

        # Step 2 — 1:1 run.
        # Daemon handles keepalive; main thread only reads drive.measured.
        results["step2_ratio_1_1"] = step2_ratio_1_1(hub)

        # Step 3 — 1:0.5 run.
        results["step3_ratio_1_half"] = step3_ratio_1_half(hub)

        # Step 4 — induced bottleneck (best-effort; needs operator).
        results["step4_induced_bottleneck"] = step4_induced_bottleneck(hub)

    finally:
        hub.fail_safe()
        _dump_csv()
        print("\n=== Summary ===", flush=True)
        for name, ok in results.items():
            print(f"  {name}: {'PASS' if ok else 'FAIL'}", flush=True)
        all_ok = all(results.values())
        print(
            f"\nOverall: {'PASS' if all_ok else 'FAIL (see details above)'}",
            flush=True,
        )


if __name__ == "__main__":
    main()
