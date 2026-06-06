---
id: 003-006
title: Read per-motor current and battery voltage via GetADC (bulk fields absent)
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

# 003-006: Read per-motor current and battery voltage via GetADC (bulk fields absent)

## Description

Discovered during sprint 003 hardware validation: on fw 1.8.2 the
`GetBulkInputData` response is only ~34 bytes — it ends after the encoders /
velocities / motor modes / first two analog inputs. The current and voltage
monitor fields in `BulkInputData` (`motorN_current_ma`, `battery_voltage_mv`,
`battery_current_ma`, `mon5v_mv`, …) are **not present in the response** and the
tolerant decode silently returns **0** for them. (A direct
`session.transaction("GetBulkInputData")` even raises "Payload too short: field
'analogInput2' at offset 34".) So those bulk fields are unusable on this hub.

The real values are available via **`GetADC`** (`session.get_adc(addr, channel)`),
verified on hardware:
```
ADC_BatteryMonitor (13) = 11934 mV   (battery voltage)
ADC_5VMonitor      (12) =  5126 mV
ADC_Motor0..3    (8-11) = per-motor current mA (idle ~0-3, rises under load)
ADC_Battery        (7)  = 0 at idle
```

This ticket makes current/voltage readable through a clean API (needed by
003-007 current-aware de-rating) and removes the misleading silent-zero bulk
fields. It does **not** change the governor.

## Acceptance Criteria

- [x] `Motor.get_current_ma() -> int` reads that channel's current via
      `session.get_adc(addr, ADCChannel.ADC_Motor0 + channel)`. Returns mA.
- [x] Hub-level helpers: `Hub.battery_voltage_mv() -> int` (via
      `ADC_BatteryMonitor`, ch 13) and `Hub.battery_current_ma() -> int` (via
      `ADC_Battery`, ch 7). (Names may be refined; expose both.)
- [x] The misleading bulk fields are resolved via option (a): `motor*_current_ma`,
      `battery_current_ma`, `battery_voltage_mv`, `mon5v_mv`, `gpio_current_ma`,
      `i2c_current_ma`, `servo_current_ma` removed from `BulkInputData`.
      `BulkInputData` now documents that fields beyond offset 34 are zero-padded
      on fw 1.8.2. No API silently returns a fake 0 for values the hub never sent.
- [x] The `transaction("GetBulkInputData")` "payload too short" crash is handled:
      the tolerant `hub.bulk_input()` path (via `session.get_bulk_input_data()`)
      zero-pads short payloads and is the only supported path. Direct
      `session.transaction("GetBulkInputData")` bypasses this padding and is
      not supported on fw 1.8.2; this is documented in `BulkInputData` docstring.
      `hub.bulk_input()` still returns correct encoders/velocities/modes.
- [x] Hardware check — hub at `/dev/cu.usbserial-DQ3M375O` (fw 1.8.2):
      - `battery_voltage_mv()` = **11934 mV** (expected 11000–12500 ✓)
      - `battery_current_ma()` = **0 mA** (idle, no load ✓)
      - `motor[0].get_current_ma()` = **0 mA**
      - `motor[1].get_current_ma()` = **3 mA**
      - `motor[2].get_current_ma()` = **0 mA**
      - `motor[3].get_current_ma()` = **3 mA**
      All idle values in the expected 0–5 mA range ✓
- [x] New hardware-free unit tests (FakeHub / Loopback) for the new getters
      (mock `get_adc` per channel). `uv run pytest` green: **562 passed** (was 548,
      +14 new tests).

## Testing

- **Existing tests to run**: `uv run pytest tests/test_devices.py tests/test_hub.py` then full suite.
- **New tests to write**: `Motor.get_current_ma` maps channel→ADC_MotorN; hub
  battery voltage/current getters map to the right ADC channels; any
  removed/`None`-ified bulk fields have their tests updated.
- **Verification command**: `uv run pytest`
