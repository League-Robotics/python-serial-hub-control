---
id: 003-004
title: Fix SerialTransport.read latency (1s per transaction)
status: done
use-cases:
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 003-004: Fix SerialTransport.read latency (1s per transaction)

## Description

Discovered during sprint 003 hardware validation (003-003): **every** RHSP
transaction takes ~1.0 s on real hardware (macOS), making the velocity_chart
tool unusable — `hub.init_peripherals()` takes ~22 s (16 transactions) and the
chart polls at ~1 Hz instead of the intended 50 Hz. This blocks SUC-002 /
SUC-003 (live convergence + phase-plot tracking).

**Root cause (proven by probe):** `SerialTransport.read(n)` calls
`pyserial.Serial.read(512)`. On macOS the configured `inter_byte_timeout=0.01`
does **not** cause an early return, so `read(512)` blocks for the full 1.0 s
`timeout` waiting for 512 bytes that never come (responses are ~20 bytes). The
session's `_read_response` therefore returns a *valid* packet but only after
~1005 ms — confirmed: 0 timeouts, 0 retries, every `_read_response` ≈ 1005 ms.

**Fix (proven: 1002 ms → 16 ms, ~60×):** read only the bytes actually waiting.
When `in_waiting > 0`, return `read(min(n, in_waiting))` immediately; when none
are waiting, do a single blocking `read(1)` (bounded by the existing `timeout`)
so the first byte still wakes us promptly without busy-waiting. The session's
existing poll loop (`if not chunk: sleep(0.001); continue`) then drains the rest
on the next iteration. Timeout/retry semantics are unchanged: if no byte ever
arrives, `read(1)` returns `b""` after `timeout` and `_read_response` still
raises `RhspTimeoutError` at its deadline.

This is a library-core change in `src/rhsp/transport.py` only. `LoopbackTransport`
(tests) already returns immediately, so hardware-free tests are unaffected.

## Acceptance Criteria

- [x] `SerialTransport.read(n)` returns as soon as data is available: when
      `self._serial.in_waiting > 0`, read at most `in_waiting` bytes (and at most
      `n`); otherwise a single bounded `read(1)`. No blind `read(512)` that
      blocks until the total timeout.
- [x] A short hardware probe shows `hub.bulk_input()` averages well under
      ~50 ms per call (was ~1000 ms). **Before fix (pre-code-change baseline):
      ~1002 ms avg (recorded at ticket creation, proven by original diagnosis).
      After fix: 15.99 ms avg (n=10), 16.01 ms avg (n=30) — confirmed on live
      hub /dev/cu.usbserial-DQ3M375O.** ~62x speedup.
- [x] Timeout behavior preserved: with no hub responding, a transaction still
      raises `RhspTimeoutError` after ~`timeout` s (not instantly, not forever).
      The `read(1)` fallback is bounded by the port's `timeout=1.0` s; the
      session layer's deadline loop raises `RhspTimeoutError` exactly as before.
- [x] `uv run pytest` stays green (540 passing, was 537 — 3 new tests added,
      zero regressions).
- [x] A regression/unit test covers the prompt-read behavior (e.g. a fake
      pyserial object exposing `in_waiting` + `read`, asserting `read()` does not
      over-block / reads available bytes). Hardware-free. Three tests added in
      `tests/test_transport.py`: `test_serial_transport_read_uses_in_waiting_path`,
      `test_serial_transport_read_in_waiting_path_respects_n`,
      `test_serial_transport_read_fallback_to_single_byte_when_empty`.

## Testing

- **Existing tests to run**: `uv run pytest` (transport + session suites).
- **New tests to write**: a hardware-free test for `SerialTransport.read` using a
  stub serial object with controllable `in_waiting` / `read`, asserting it
  returns available bytes promptly and falls back to a single `read(1)` when the
  buffer is empty.
- **Verification command**: `uv run pytest`
- **Hardware probe** (run by team-lead/programmer; no motor motion): connect and
  time ~30 `hub.bulk_input()` calls; expect avg < 50 ms.
