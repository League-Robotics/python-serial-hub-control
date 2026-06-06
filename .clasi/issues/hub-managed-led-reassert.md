---
status: pending
---

# Hub-managed LED: re-assert colour in keep-alive heartbeat and clear latched status on connect

Make `hub.set_led_color()` fire-and-forget so a user-set LED colour stays
steady without the caller manually re-sending the pattern.

## Context / why

Sprint 001 shipped working LED control, verified on hub firmware 1.8.2
(commit `561099a`, see the `led-control-recipe` project memory):

- pattern step wire order is `[T, B, G, R]` (duration, blue, green, red),
- the latched `KeepAliveTimeout | FailSafe` status must be cleared before the
  hub will obey LED commands,
- the LED is driven via `SetModuleLEDPattern` (0x7F0C); `SetModuleLEDColor`
  (0x7F0A) is a confirmed no-op on this firmware.

But `hub.set_led_color()` is currently a **one-shot**. The colour only stays
steady while the keep-alive heartbeat is running, and if the firmware
re-latches a timeout (or a device reset occurs) the LED reverts to its
blinking-blue status animation. During hardware testing we had to manually
re-send the pattern every ~0.8 s to keep it rock-steady. The library should do
this for the user.

## Scope of the follow-up

- **Heartbeat re-assert.** Have the Hub keep-alive heartbeat thread remember
  the desired LED colour/pattern and re-assert it each tick. This mirrors the
  FTC SDK's `resendCurrentPattern()` after a keep-alive timeout / device reset.
- **Clear status on connect.** Clear the latched `KeepAliveTimeout | FailSafe`
  as part of connect/init so the LED goes straight to the user colour (or solid
  connected-green) instead of blinking blue during normal operation.
- **API decision.** `set_led_color()` stores the desired colour on the Hub; the
  heartbeat re-asserts it; add a way to clear it back to firmware-default.
  Consider a `set_led_pattern()` passthrough for multi-step animations too.
- **Don't mask real faults.** Re-clearing status every heartbeat could hide a
  genuine `KeepAliveTimeout` / `FailSafe` / over-temp / battery-low event. Only
  clear/re-assert when needed, and surface real faults rather than swallowing
  them.
- **Tests.** Extend the FakeHub-based suite to assert the heartbeat re-asserts
  the stored pattern and that status is cleared on connect. Keep the
  hardware-free pytest suite as the gate; validate on the live hub per the
  `test-on-real-hardware` directive.

## References

- `led-control-recipe` project memory (the verified recipe)
- fix commit `561099a`
- `session.set_module_led_pattern`, `hub.set_led_color`, `Hub.start_keepalive`
