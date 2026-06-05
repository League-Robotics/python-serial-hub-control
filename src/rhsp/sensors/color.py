"""Color sensor drivers — APDS-9960 (ColorSensor) and REV Color Sensor V3 (ColorSensorV3).

Both drivers are layered on :class:`~rhsp.devices.i2c.I2CDevice` and use
``write_register`` / ``read_register`` exclusively.  They do NOT subclass
``I2CDevice``; they accept one as a constructor argument.

Ported faithfully from ``vendor/rhsp/color.py`` with the following fixes:
- No ``ord()`` / ``readByte`` crash paths — all reads go through ``read_register``
  which returns clean ``bytes``.
- ``ColorSensor.__init__`` performs init sequence in :meth:`__init__`, not a
  separate ``initSensor()`` method; raises ``ProtocolError`` on bad device ID.
- Clean snake_case public API.
"""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING

from rhsp.errors import ProtocolError
from rhsp.sensors.registers import (
    # APDS-9960
    COMMAND_BIT,
    MULTI_BYTE_BIT,
    APDS9960_ENABLE,
    APDS9960_ATIME,
    APDS9960_PPULSE,
    APDS9960_ID,
    APDS9960_CDATAL,
    APDS9960_DEVICE_ID,
    # APDS-9151 (REV Color Sensor V3)
    APDS9151_MAIN_CTRL,
    APDS9151_PS_PULSES,
    APDS9151_PS_MEAS_RATE,
    APDS9151_LS_GAIN,
    APDS9151_PART_ID,
    APDS9151_PS_DATA,
    APDS9151_LS_DATA_IR_0,
    APDS9151_DEVICE_ID,
    APDS9151_PS_EN,
    APDS9151_LS_EN,
    APDS9151_RGB_MODE,
    APDS9151_PS_MEAS_RATE_100ms,
    APDS9151_PS_RES_11_BIT,
    APDS9151_LS_GAIN_9,
)

if TYPE_CHECKING:
    from rhsp.devices.i2c import I2CDevice

__all__ = ["ColorSensor", "ColorSensorV3"]


class ColorSensor:
    """Driver for the APDS-9960 color / proximity sensor.

    Performs the initialization sequence in ``__init__``:
    1. Write ENABLE register (PON + AEN + PEN = 0x07).
    2. Write ATIME register (0xFF — max integration time).
    3. Write PPULSE register (0x08).
    4. Read device ID; raise :exc:`~rhsp.errors.ProtocolError` if not 0x60.

    Parameters
    ----------
    device:
        :class:`~rhsp.devices.i2c.I2CDevice` bound to the sensor's I2C address
        and channel.  Typically created via ``hub.i2c[1].device(0x39)``.

    Raises
    ------
    ProtocolError
        If the device ID register does not return 0x60.
    """

    def __init__(self, device: "I2CDevice") -> None:
        self._dev = device
        self._init()

    def _init(self) -> None:
        """Run the APDS-9960 initialization sequence."""
        # Enable register: PON (bit0) + AEN (bit1) + PEN (bit2) = 0x07
        self._dev.write_register(
            COMMAND_BIT | APDS9960_ENABLE, bytes([0x07])
        )
        # ATIME register: 0xFF (maximum integration time)
        self._dev.write_register(
            COMMAND_BIT | APDS9960_ATIME, bytes([0xFF])
        )
        # PPULSE register: 0x08
        self._dev.write_register(
            COMMAND_BIT | APDS9960_PPULSE, bytes([0x08])
        )
        # Read device ID — use COMMAND_BIT | MULTI_BYTE_BIT per vendor protocol
        id_bytes = self._dev.read_register(
            COMMAND_BIT | MULTI_BYTE_BIT | APDS9960_ID, 1
        )
        device_id = id_bytes[0]
        if device_id != APDS9960_DEVICE_ID:
            raise ProtocolError(
                f"unexpected color sensor id: got 0x{device_id:02X}, "
                f"expected 0x{APDS9960_DEVICE_ID:02X}"
            )

    def read_color(self) -> tuple[int, int, int, int]:
        """Read RGBC values from the sensor.

        Issues a single multi-byte read starting at ``CDATAL`` (8 bytes for
        4 × 16-bit LE channels: clear, red, green, blue).

        Returns
        -------
        tuple[int, int, int, int]
            ``(red, green, blue, clear)`` as 16-bit unsigned integers.
        """
        raw = self._dev.read_register(
            COMMAND_BIT | MULTI_BYTE_BIT | APDS9960_CDATAL, 8
        )
        # 8 bytes: CDATAL[0:2], RDATAL[2:4], GDATAL[4:6], BDATAL[6:8]
        clear, red, green, blue = struct.unpack_from("<HHHH", raw, 0)
        return (red, green, blue, clear)

    def __repr__(self) -> str:
        return f"ColorSensor(device={self._dev!r})"


class ColorSensorV3:
    """Driver for the REV Color Sensor V3 (APDS-9151).

    Performs the initialization sequence in ``__init__``:
    1. Read PART_ID; return ``False`` silently if wrong device ID.
    2. Write MAIN_CTRL (RGB mode + LS enable + PS enable).
    3. Write PS_PULSES (32).
    4. Write PS_MEAS_RATE (11-bit resolution + 100 ms rate).
    5. Write LS_GAIN (gain × 9).

    Parameters
    ----------
    device:
        :class:`~rhsp.devices.i2c.I2CDevice` bound to the sensor at address
        0x52 (82 decimal).
    """

    def __init__(self, device: "I2CDevice") -> None:
        self._dev = device
        self._initialized = self._init()

    def _init(self) -> bool:
        """Run the APDS-9151 initialization sequence.

        Returns
        -------
        bool
            ``True`` if initialization succeeded (correct device ID found),
            ``False`` otherwise.
        """
        try:
            id_bytes = self._dev.read_register(APDS9151_PART_ID, 1)
            ident = id_bytes[0]
        except Exception:
            return False

        if ident != APDS9151_DEVICE_ID:
            return False

        # MAIN_CTRL: RGB mode | LS enable | PS enable
        self._dev.write_register(
            APDS9151_MAIN_CTRL,
            bytes([APDS9151_RGB_MODE | APDS9151_LS_EN | APDS9151_PS_EN]),
        )
        # PS_PULSES: 32
        self._dev.write_register(APDS9151_PS_PULSES, bytes([32]))
        # PS_MEAS_RATE: 11-bit resolution + 100 ms rate
        self._dev.write_register(
            APDS9151_PS_MEAS_RATE,
            bytes([APDS9151_PS_RES_11_BIT | APDS9151_PS_MEAS_RATE_100ms]),
        )
        # LS_GAIN: gain × 9
        self._dev.write_register(APDS9151_LS_GAIN, bytes([APDS9151_LS_GAIN_9]))
        return True

    @property
    def initialized(self) -> bool:
        """True if the sensor was found and initialized successfully."""
        return self._initialized

    def read_all(self) -> tuple[int, int, int, int, int]:
        """Read proximity, red, green, blue, and IR values as a block.

        Issues a block read starting at ``PS_DATA`` (14 bytes).

        Returns
        -------
        tuple[int, int, int, int, int]
            ``(red, green, blue, ir, proximity)`` as raw integer values.
        """
        raw = self._dev.read_register(APDS9151_PS_DATA, 14)
        # Proximity: 11-bit value at bytes 0:2
        prox = int.from_bytes(raw[0:2], "little") & 0x7FF
        # IR: 24-bit at bytes 2:5
        ir = int.from_bytes(raw[2:5], "little") & 0xFFFFFF
        # Green: 24-bit at bytes 5:8
        green = int.from_bytes(raw[5:8], "little") & 0xFFFFFF
        # Blue: 24-bit at bytes 8:11
        blue = int.from_bytes(raw[8:11], "little") & 0xFFFFFF
        # Red: 24-bit at bytes 11:14
        red = int.from_bytes(raw[11:14], "little") & 0xFFFFFF
        return (red, green, blue, ir, prox)

    def read_color(self) -> tuple[int, int, int, int]:
        """Read RGBC (red, green, blue, clear/calculated) values.

        Returns
        -------
        tuple[int, int, int, int]
            ``(red, green, blue, clear)`` where clear is calculated as
            ``r + g + b - 2 * ir``.
        """
        raw = self._dev.read_register(APDS9151_LS_DATA_IR_0, 12)
        ir = int.from_bytes(raw[0:3], "little") & 0xFFFFFF
        green = int.from_bytes(raw[3:6], "little") & 0xFFFFFF
        blue = int.from_bytes(raw[6:9], "little") & 0xFFFFFF
        red = int.from_bytes(raw[9:12], "little") & 0xFFFFFF
        clear = red + green + blue - 2 * ir
        return (red, green, blue, clear)

    def __repr__(self) -> str:
        return f"ColorSensorV3(device={self._dev!r})"
