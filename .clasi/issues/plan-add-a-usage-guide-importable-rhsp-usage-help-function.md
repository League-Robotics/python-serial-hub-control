---
status: pending
---

# Plan: Add a usage guide + importable `rhsp.usage()` help function

## Context

The `rhsp` library (pure-Python REV Hub Serial Protocol driver) has a short
Quick-start in the README but no complete, self-contained usage guide covering
device setup and every operation. The user wants:

1. A **comprehensive documentation file** — how to set up a device, how to run
   things, with examples.
2. A **version bundled inside the library** that agents can retrieve at runtime,
   so an agent can write a one-liner to get usage help without reading the repo.
3. A **README reference** explaining the importable help function + the one-liner.

**Decisions (confirmed with user):**
- **Single source of truth.** One guide lives at `src/rhsp/USAGE.md`. The
  `usage()` function returns its text; the README and `docs/` link to the same
  file. No duplicate content, no drift.
- **Function name: `usage()`.** Agent one-liner:
  `python -c "import rhsp; print(rhsp.usage())"`.

**Why `src/rhsp/USAGE.md` (not `docs/`):** the hatchling build auto-includes
non-Python files under `src/rhsp/` in the wheel, and
`importlib.resources.files("rhsp")` resolves to `src/rhsp/` in **both** editable
(`uv sync`) and installed-wheel modes. Putting the file there is the only
location where `usage()` works in dev and after `pip install` with zero
`pyproject.toml` changes. This mirrors the existing
[catalogue.py](src/rhsp/catalogue.py) pattern that reads bundled `protocol.json`.

## Changes

### 1. New file: `src/rhsp/USAGE.md` (the single-source guide)

A complete, example-driven Markdown guide. Content is grounded in the real API
verified during exploration — `connect`/`enumerate_hubs` in
[discovery.py](src/rhsp/discovery.py), the `Hub` class in [hub.py](src/rhsp/hub.py),
device classes in [src/rhsp/devices/](src/rhsp/devices/), and the runnable
[examples/](examples/). Sections:

- **Overview** — what the library does; the `rhsp` package; Python ≥ 3.13.
- **Installation** — `pip install` / `uv pip install` from git; `uv sync` for dev.
- **Setting up a device** — `enumerate_hubs()`, `connect(port, *, baud=460_800,
  timeout=1.0, retries=3)`, the **context-manager** pattern (auto keep-alive +
  `fail_safe()` on exit), and `hub.init_peripherals()`. Explain the keep-alive
  heartbeat requirement (fw 1.8.2 disables outputs after ~2500 ms without one)
  and `start_keepalive`/`stop_keepalive` for manual control.
- **Hub object layout** — `hub.motors[0..3]`, `hub.servos[0..5]`,
  `hub.dio[0..7]`, `hub.adc[0..3]`, `hub.i2c[0..3]` + `.device(addr)`.
- **Running things** — copy-pasteable examples per subsystem:
  - Motors: `set_mode(MotorMode.CONSTANT_POWER, ...)`, `enable`/`disable`,
    `set_power` (-32767..32767); velocity (`set_target_velocity`,
    `get_velocity(bulk)`); position (`set_target_position`, `at_target`);
    encoder (`get_encoder_position`, `reset_encoder`); `get_current_ma`;
    `set_velocity_pid`/`get_velocity_pid`.
  - Servos: `set_configuration(20_000)`, `enable`, `set_angle(0..180)`,
    `set_pulse_width(500..2500)`.
  - Digital I/O: `set_direction(output=True)`, `write`, `read`.
  - ADC + battery: `adc.read(raw=False)`, `hub.battery_voltage_mv()`,
    `hub.battery_current_ma()`.
  - Hub LED: `set_led_color(r,g,b)`, `clear_led_color()` (note the fw 1.8.2
    pattern-vs-color quirk, consistent with the LED-control memory).
  - I2C sensors: `ColorSensorV3` and `Distance2m` from
    [src/rhsp/sensors/](src/rhsp/sensors/); bulk IMU via `hub.bulk_input()`.
  - Bulk reads: `hub.bulk_input()` to snapshot all motors at once.
  - Higher-level helpers: `HubVelocityController` / `RatioDrive` from
    [control](src/rhsp/control.py) (already exported).
- **Full end-to-end example** — a single runnable script (discover → connect →
  init → drive motor/servo/DIO → read sensor → clean shutdown).
- **Status/diagnostics** — `get_module_status()`, `read_version_string()`,
  `fail_safe()`.
- **Hardware notes & gotchas** — 460_800 baud / REV serial-number detection;
  power range sentinels; LED quirk; bulk-input field caveats; actuators need the
  12 V battery.
- **Getting more help** — pointer to `docs/RHSP-Protocol.md` and the runnable
  examples; note `python -c "import rhsp; print(rhsp.usage())"`.

### 2. Edit `src/rhsp/__init__.py` — add `usage()`

Add a function that reads the bundled guide via the same mechanism already used
in `catalogue.py`:

```python
import importlib.resources

def usage() -> str:
    """Return the bundled rhsp usage guide (USAGE.md) as a string.

    Handy for agents/REPLs:
        python -c "import rhsp; print(rhsp.usage())"
    """
    return (
        importlib.resources.files("rhsp")
        .joinpath("USAGE.md")
        .read_text(encoding="utf-8")
    )
```

- Add `"usage"` to `__all__`.
- Mention `usage()` in the module docstring's "Public API surface" list.

### 3. Edit `README.md` — reference the guide and the one-liner

- Add a short **"Usage guide"** / **"Getting help"** section (right after
  Quick start, ~line 90) linking to [src/rhsp/USAGE.md](src/rhsp/USAGE.md) and
  showing the agent one-liner:
  ```bash
  python -c "import rhsp; print(rhsp.usage())"
  ```
  plus the REPL form `import rhsp; print(rhsp.usage())`.
- Add `USAGE.md` to the `src/rhsp/` entry in the **Layout** block.

## Reuse / consistency notes

- `usage()` reuses the **exact** `importlib.resources.files("rhsp").joinpath(...)`
  pattern from [catalogue.py](src/rhsp/catalogue.py) — no new dependency, no
  `pyproject.toml` change (hatchling already ships `protocol.json` and `py.typed`
  the same way).
- Examples in the guide are copied/adapted from the verified, runnable scripts in
  [examples/](examples/) so they stay accurate.

## Verification

1. **Editable mode works:**
   `uv run python -c "import rhsp; print(rhsp.usage()[:200])"` prints the guide
   header (proves `importlib.resources` resolves the file from `src/rhsp/`).
2. **Export present:** `uv run python -c "import rhsp; assert 'usage' in rhsp.__all__ and callable(rhsp.usage)"`.
3. **Wheel ships the file:** `uv build` then inspect the wheel
   (`python -c "import zipfile,glob; print([n for n in zipfile.ZipFile(glob.glob('dist/*.whl')[0]).namelist() if n.endswith('USAGE.md')])"`)
   to confirm `rhsp/USAGE.md` is included.
4. **Tests still green:** `uv run pytest tests/` (hardware-free).
5. **Spot-check accuracy:** confirm every method named in the guide exists in the
   current source (grep the symbols against `src/rhsp/`).
6. **Optional (hardware):** on the real hub
   (`/dev/cu.usbserial-DQ3M375O`), run the guide's end-to-end example with the
   12 V battery attached to confirm the examples actually drive the device.
