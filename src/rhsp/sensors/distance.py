"""Distance sensor driver — VL53L0X / REV 2m Distance Sensor (Distance2m).

Layered strictly on :class:`~rhsp.devices.i2c.I2CDevice`.  Uses
``write_register`` / ``read_register`` exclusively — no ``ord()`` or raw
``readByte()`` calls.

Faithfully ports the ST VL53L0X initialization sequence from
``vendor/rhsp/distance.py`` with the following fixes:

P7-f: ``OSC_CALIBRATE_VAL`` imported from ``registers.py`` (0xF8).  The
vendor code referenced ``OSC_CALIBRATE_VAL`` as a bare name that was never
defined at module scope, causing a ``NameError`` at runtime.

Clean ``int`` reads: All ``readByte()`` / ``readMultipleBytes()`` paths are
replaced with ``read_register(addr, n)`` which returns ``bytes``; integer
values are extracted with ``data[0]`` or ``int.from_bytes(data, 'little')``.
"""

from __future__ import annotations

import struct
import time
from typing import TYPE_CHECKING

from rhsp.sensors.registers import (
    OSC_CALIBRATE_VAL,
    SYSRANGE_START,
    SYSTEM_SEQUENCE_CONFIG,
    SYSTEM_INTERRUPT_CONFIG_GPIO,
    GPIO_HV_MUX_ACTIVE_HIGH,
    SYSTEM_INTERRUPT_CLEAR,
    RESULT_INTERRUPT_STATUS,
    RESULT_RANGE_STATUS,
    MSRC_CONFIG_CONTROL,
    MSRC_CONFIG_TIMEOUT_MACROP,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    PRE_RANGE_CONFIG_VCSEL_PERIOD,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
    GLOBAL_CONFIG_REF_EN_START_SELECT,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD,
    DYNAMIC_SPAD_REF_EN_START_OFFSET,
)

if TYPE_CHECKING:
    from rhsp.devices.i2c import I2CDevice

__all__ = ["Distance2m"]

# VcselPeriod type constants
_VCSEL_PERIOD_PRE_RANGE: int = 0
_VCSEL_PERIOD_FINAL_RANGE: int = 1


class Distance2m:
    """Driver for the VL53L0X / REV 2m Distance Sensor.

    Runs the full ST initialization sequence in :meth:`__init__`.  Call
    :meth:`read_mm` to obtain a range reading in millimeters.

    Parameters
    ----------
    device:
        :class:`~rhsp.devices.i2c.I2CDevice` bound to the sensor's I2C address
        (0x29) and channel.  Typically created via ``hub.i2c[0].device(0x29)``.
    """

    def __init__(self, device: "I2CDevice") -> None:
        self._dev = device
        self._stop_variable: int = 0
        self._spad_count: int = 0
        self._spad_type_is_aperture: bool = False
        self._io_timeout: int = 0
        # One read to verify the device is present before the 80-step ST init.
        # When the I2C channel has SDA stuck, each operation causes a hub-side
        # I2C timeout; catching it here avoids 80× that delay.
        model_id = self._dev.read_register(0xC0, 1)[0]
        if model_id != 0xEE:
            from rhsp.errors import ProtocolError
            raise ProtocolError(
                f"VL53L0X not present: expected model id 0xEE at reg 0xC0, "
                f"got 0x{model_id:02X}"
            )
        self.initialize()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _read1(self, addr: int) -> int:
        """Read one byte from *addr* and return it as int."""
        return self._dev.read_register(addr, 1)[0]

    def _read2(self, addr: int) -> int:
        """Read two bytes from *addr* and return big-endian unsigned int."""
        data = self._dev.read_register(addr, 2)
        return (data[0] << 8) | data[1]

    def _write1(self, addr: int, value: int) -> None:
        """Write a single byte *value* to *addr*."""
        self._dev.write_register(addr, bytes([value]))

    def _write2(self, addr: int, value: int) -> None:
        """Write a big-endian 16-bit *value* to *addr*."""
        self._dev.write_register(addr, bytes([(value >> 8) & 0xFF, value & 0xFF]))

    # ------------------------------------------------------------------
    # Initialization sequence (§2.14 ST bring-up recipe)
    # ------------------------------------------------------------------

    def initialize(self) -> bool:
        """Run the full VL53L0X ST initialization sequence.

        Returns
        -------
        bool
            ``True`` on success.
        """
        self._write1(0x88, 0)
        self._write1(0x80, 1)
        self._write1(0xFF, 1)
        self._write1(0x00, 0)
        self._stop_variable = self._read1(0x91)
        self._write1(0x00, 1)
        self._write1(0xFF, 0)
        self._write1(0x80, 0)

        # Disable MSRC and TCC by default
        msrc_ctrl = self._read1(MSRC_CONFIG_CONTROL) | 0x12
        self._write1(MSRC_CONFIG_CONTROL, msrc_ctrl)

        # Set signal rate limit to 0.25 Mcps (0.25 * 128 = 32)
        self._set_signal_rate_limit(0.25)

        self._write1(SYSTEM_SEQUENCE_CONFIG, 0xFF)

        if not self._get_spad_info():
            return False

        # Read and modify SPAD reference map
        ref_spad_map = list(self._dev.read_register(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6))

        self._write1(0xFF, 1)
        self._write1(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0)
        self._write1(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 44)
        self._write1(0xFF, 0)
        self._write1(GLOBAL_CONFIG_REF_EN_START_SELECT, 180)

        first_spad_to_enable = 12 if self._spad_type_is_aperture else 0
        spads_enabled = 0
        for i in range(48):
            tmp_idx = i // 8
            if i < first_spad_to_enable or spads_enabled == self._spad_count:
                ref_spad_map[tmp_idx] &= ~(1 << (i % 8))
            elif ref_spad_map[tmp_idx] >> (i % 8) & 1:
                spads_enabled += 1

        self._dev.write_register(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, bytes(ref_spad_map))

        # Load default tuning register values (ST data-init sequence)
        self._write1(0xFF, 1)
        self._write1(0x00, 0)
        self._write1(0xFF, 0)
        self._write1(0x09, 0)
        self._write1(0x10, 0)
        self._write1(0x11, 0)
        self._write1(0x24, 1)
        self._write1(0x25, 0xFF)
        self._write1(0x75, 0)
        self._write1(0xFF, 1)
        self._write1(0x4E, 44)
        self._write1(0x48, 0)
        self._write1(0x30, 32)
        self._write1(0xFF, 0)
        self._write1(0x30, 9)
        self._write1(0x54, 0)
        self._write1(0x31, 4)
        self._write1(0x32, 3)
        self._write1(0x40, 0x83)
        self._write1(0x46, 37)
        self._write1(0x60, 0)
        self._write1(0x27, 0)
        self._write1(0x50, 6)
        self._write1(0x51, 0)
        self._write1(0x52, 0x96)
        self._write1(0x56, 8)
        self._write1(0x57, 0x30)
        self._write1(0x61, 0)
        self._write1(0x62, 0)
        self._write1(0x64, 0)
        self._write1(0x65, 0)
        self._write1(0x66, 0xA0)
        self._write1(0xFF, 1)
        self._write1(0x22, 50)
        self._write1(0x47, 20)
        self._write1(0x49, 0xFF)
        self._write1(0x4A, 0)
        self._write1(0xFF, 0)
        self._write1(0x7A, 10)
        self._write1(0x7B, 0)
        self._write1(0x78, 33)
        self._write1(0xFF, 1)
        self._write1(0x23, 52)
        self._write1(0x42, 0)
        self._write1(0x44, 0xFF)
        self._write1(0x45, 38)
        self._write1(0x46, 5)
        self._write1(0x40, 0x40)
        self._write1(0x0E, 6)
        self._write1(0x20, 26)
        self._write1(0x43, 0x40)
        self._write1(0xFF, 0)
        self._write1(0x34, 3)
        self._write1(0x35, 68)
        self._write1(0xFF, 1)
        self._write1(0x31, 4)
        self._write1(0x4B, 9)
        self._write1(0x4C, 5)
        self._write1(0x4D, 4)
        self._write1(0xFF, 0)
        self._write1(0x44, 0)
        self._write1(0x45, 32)
        self._write1(0x47, 8)
        self._write1(0x48, 40)
        self._write1(0x67, 0)
        self._write1(0x70, 4)
        self._write1(0x71, 1)
        self._write1(0x72, 0xFE)
        self._write1(0x76, 0)
        self._write1(0x77, 0)
        self._write1(0xFF, 1)
        self._write1(0x0D, 1)
        self._write1(0xFF, 0)
        self._write1(0x80, 1)
        # P7-f: OSC_CALIBRATE_VAL = 0xF8 — vendor had this undefined
        self._write1(0x01, OSC_CALIBRATE_VAL)
        self._write1(0xFF, 1)
        self._write1(0x8E, 1)
        self._write1(0x00, 1)
        self._write1(0xFF, 0)
        self._write1(0x80, 0)

        # Configure GPIO interrupt (new sample ready on interrupt pin)
        self._write1(SYSTEM_INTERRUPT_CONFIG_GPIO, 4)
        gpio_hv = self._read1(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10
        self._write1(GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv)
        self._write1(SYSTEM_INTERRUPT_CLEAR, 1)

        # Perform reference calibration
        self._write1(SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self._write1(SYSTEM_SEQUENCE_CONFIG, 0x01)
        if not self._perform_single_ref_calibration(0x40):
            return False

        self._write1(SYSTEM_SEQUENCE_CONFIG, 0x02)
        if not self._perform_single_ref_calibration(0x00):
            return False

        self._write1(SYSTEM_SEQUENCE_CONFIG, 0xE8)

        self.set_timeout(200)
        self._start_continuous()
        return True

    def _get_spad_info(self) -> bool:
        """Read SPAD info (count and aperture type) from the sensor."""
        self._write1(0x80, 1)
        self._write1(0xFF, 1)
        self._write1(0x00, 0)
        self._write1(0xFF, 0x06)
        tmp = self._read1(0x83) | 4
        self._write1(0x83, tmp)
        self._write1(0xFF, 0x07)
        self._write1(0x81, 1)
        self._write1(0x80, 1)
        self._write1(0x94, 0x6B)
        self._write1(0x83, 0)
        self._write1(0x83, 1)
        tmp = self._read1(0x92)
        self._spad_count = tmp & 0x7F
        self._spad_type_is_aperture = bool((tmp >> 7) & 1)
        self._write1(0x81, 0)
        self._write1(0xFF, 0x06)
        tmp = self._read1(0x83) & ~4
        self._write1(0x83, tmp)
        self._write1(0xFF, 1)
        self._write1(0x00, 1)
        self._write1(0xFF, 0)
        self._write1(0x80, 0)
        return True

    def _set_signal_rate_limit(self, limit_mcps: float) -> None:
        """Set signal rate limit in units of Mcps (mega-counts per second)."""
        encoded = int(limit_mcps * 128)
        self._write2(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, encoded)

    def _perform_single_ref_calibration(self, vhv_init_byte: int) -> bool:
        """Run a single VHV / phase calibration pass."""
        self._write1(SYSRANGE_START, 0x01 | vhv_init_byte)
        self._write1(SYSTEM_INTERRUPT_CLEAR, 1)
        self._write1(SYSRANGE_START, 0)
        return True

    def _start_continuous(self, period_ms: int = 0) -> None:
        """Start continuous ranging mode."""
        self._write1(0x80, 1)
        self._write1(0xFF, 1)
        self._write1(0x00, 0)
        self._write1(0x91, self._stop_variable)
        self._write1(0x00, 1)
        self._write1(0xFF, 0)
        self._write1(0x80, 0)
        if period_ms != 0:
            osc_cal = self._read1(OSC_CALIBRATE_VAL)
            if osc_cal != 0:
                period_ms *= osc_cal
            self._write1(SYSRANGE_START, 0x04)
        else:
            self._write1(SYSRANGE_START, 0x02)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_timeout(self, timeout_ms: int) -> None:
        """Set the read timeout in milliseconds."""
        self._io_timeout = timeout_ms

    def is_present(self) -> bool:
        """Check the identification register to confirm a VL53L0X is present.

        Returns
        -------
        bool
            ``True`` if the model-id register returns 0xEE.
        """
        try:
            model_id = self._read1(0xC0)
            return model_id == 0xEE
        except Exception:
            return False

    def read_mm(self) -> int:
        """Read the current range in millimeters (continuous mode).

        Polls the interrupt status register until a new measurement is
        available (up to :attr:`_io_timeout` milliseconds), then reads
        the range result and clears the interrupt.

        Returns
        -------
        int
            Range in millimeters, or 65535 on timeout.
        """
        if self._io_timeout > 0:
            start = time.time()

        while (self._read1(RESULT_INTERRUPT_STATUS) & 0x07) == 0:
            if self._io_timeout > 0:
                elapsed_ms = (time.time() - start) * 1000
                if elapsed_ms > self._io_timeout:
                    return 65535

        # Range result is at RESULT_RANGE_STATUS + 10 (2 bytes, big-endian)
        range_data = self._dev.read_register(RESULT_RANGE_STATUS + 10, 2)
        range_mm = (range_data[0] << 8) | range_data[1]

        # Clear interrupt
        self._write1(SYSTEM_INTERRUPT_CLEAR, 1)
        return range_mm

    def __repr__(self) -> str:
        return f"Distance2m(device={self._dev!r})"
