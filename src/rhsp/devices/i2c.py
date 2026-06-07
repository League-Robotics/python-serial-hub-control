"""I2CChannel and I2CDevice — typed wrappers for RHSP I2C commands.

Wire-protocol notes (verified on REV Expansion Hub fw 1.8.2)
-------------------------------------------------------------
**Register-pointer write**: use ``I2CWriteSingleByte`` (command 0x1025) to
set the device's internal register pointer.  ``I2CWriteMultipleBytes`` with
a single-byte payload does **not** reliably set the pointer on this firmware
— only ``WriteSingleByte`` does.  ``WriteMultipleBytes`` is correct for
register writes that include a data payload (≥2 bytes: [reg_addr, data...]).

**Single-byte read**: use ``I2CReadSingleByte`` (command 0x1027) to read
exactly one byte.  ``I2CReadMultipleBytes(numBytes=1)`` returns a stale
result from the hub's internal buffer on this firmware; single-byte reads
MUST go through ``I2CReadSingleByte``.

**Multi-byte read** (N ≥ 2): ``I2CReadMultipleBytes`` works correctly.

**I2C status in ``I2CReadStatusQuery_RSP``**: the hub signals flow control
via the ``i2cStatus`` byte:
- 0  = success / ACK
- 3  = NACK (device not present or bus error)
- 41 = operation in progress (poll again)
- 46 = I2C bus timeout

**Auto-configure**: an :class:`I2CChannel` configures itself to
``STANDARD_100KHZ`` the first time a :class:`I2CDevice` is created (via
:meth:`I2CChannel.device`), so callers need not call
:meth:`I2CChannel.configure` manually.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from rhsp.errors import I2CError, RhspTimeoutError
from rhsp.enums import I2CSpeedCode

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["I2CChannel", "I2CDevice"]

# I2C status code that means "still in progress — poll again".
_I2C_IN_PROGRESS: int = 41

# I2C status code that means device NACKed (not present / bus error).
_I2C_NACK: int = 3

# I2C status code that means "bus timeout" (device did not respond in time).
_I2C_TIMEOUT: int = 46

# Maximum number of read-status polls before raising RhspTimeoutError.
_MAX_POLLS: int = 20

# Delay between polls (seconds).
_POLL_DELAY: float = 0.005

# Default speed when auto-configuring a channel on first device creation.
_DEFAULT_SPEED: int = I2CSpeedCode.STANDARD_100KHZ


class I2CChannel:
    """Typed representation of a single I2C channel.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    channel:
        I2C channel index (0–3).
    address:
        Hub module address on the RS-485 bus.
    """

    def __init__(self, session: "Session", channel: int, address: int) -> None:
        self.session = session
        self.channel = channel
        self.address = address
        # Track whether configure() has been called so we can auto-configure.
        self._configured: bool = False

    def configure(self, speed_code: int) -> None:
        """Configure the I2C bus speed for this channel.

        Parameters:
            speed_code: Speed code (see :class:`~rhsp.enums.I2CSpeedCode`).
        """
        self.session.i2c_configure_channel(self.address, self.channel, speed_code)
        self._configured = True

    def device(self, address: int) -> "I2CDevice":
        """Create an :class:`I2CDevice` for an I2C peripheral at *address*.

        Auto-configures the channel to ``STANDARD_100KHZ`` on first call if
        :meth:`configure` has not been called yet, so callers do not need to
        remember to configure the channel before creating devices.

        Parameters:
            address: 7-bit I2C device address.

        Returns:
            An :class:`I2CDevice` bound to this channel.
        """
        if not self._configured:
            self.configure(_DEFAULT_SPEED)
        return I2CDevice(self.session, self.channel, self.address, address)

    def __repr__(self) -> str:
        return f"I2CChannel(address={self.address!r}, channel={self.channel!r})"


class I2CDevice:
    """Typed wrapper for a single I2C peripheral device.

    Provides register-level read/write operations over the RHSP I2C
    transaction protocol.

    Wire protocol (verified on fw 1.8.2)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    **Read N bytes from register R**:

    1. ``I2CWriteSingleByte(R)`` — set the device register pointer.
    2. ``I2CReadSingleByte()`` (N=1) **or** ``I2CReadMultipleBytes(N)`` (N≥2)
       — initiate the read.
    3. Poll ``I2CReadStatusQuery`` until ``i2cStatus == 0`` (success).

    **Write bytes to register R**:

    - ``I2CWriteMultipleBytes([R, data...])`` — register address as the first
      byte of the payload, followed by data.  This requires a payload of at
      least 2 bytes (register + at least 1 data byte).

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    i2c_channel:
        I2C channel index (0–3) on the hub.
    dest:
        Hub module address on the RS-485 bus.
    address:
        7-bit I2C device address.
    """

    def __init__(
        self,
        session: "Session",
        i2c_channel: int,
        dest: int,
        address: int,
    ) -> None:
        self.session = session
        self.i2c_channel = i2c_channel
        self.dest = dest
        self.address = address

    def write_register(self, register: int, data: bytes) -> None:
        """Write bytes to a device register.

        Encodes the register byte followed by *data* into a single
        ``I2CWriteMultipleBytes`` transaction (total ≥ 2 bytes on the I2C
        bus — the register address plus the data payload).

        Parameters:
            register: Register address (0–255).
            data:     Bytes to write (at least 1 byte).

        Raises:
            ValueError: If *data* is empty.
            I2CError:   If the device NACKs the write.
        """
        if not data:
            raise ValueError(
                "write_register: data must contain at least one byte. "
                "Use i2c_write_single_byte to set a register pointer with no data."
            )
        payload = bytes([register]) + bytes(data)
        self.session.i2c_write_multiple_bytes(
            self.dest, self.i2c_channel, self.address, payload
        )

    def read_register(self, register: int, num_bytes: int) -> bytes:
        """Read bytes from a device register.

        Wire protocol (verified on fw 1.8.2):

        1. ``I2CWriteSingleByte(register)`` — set the device register pointer.
        2. ``I2CReadSingleByte()`` (num_bytes=1) **or**
           ``I2CReadMultipleBytes(num_bytes)`` (num_bytes≥2) — initiate read.
        3. Poll ``I2CReadStatusQuery`` until the hub reports data.

        Parameters:
            register:  Register address (0–255).
            num_bytes: Number of bytes to read (≥1).

        Returns:
            The bytes read from the device.

        Raises:
            I2CError:         If the device NACKs (``i2cStatus=3``) or a bus
                              timeout occurs (``i2cStatus=46``).
            RhspTimeoutError: If the hub does not complete the read within the
                              poll limit.
        """
        # Phase 1: write the register pointer using WriteSingleByte.
        # (WriteMultipleBytes with 1 byte does not reliably set the pointer on
        # fw 1.8.2 — verified by hardware testing.)
        self.session.i2c_write_single_byte(
            self.dest, self.i2c_channel, self.address, register
        )
        # Phase 2: initiate the read.
        if num_bytes == 1:
            # ReadSingleByte is required for 1-byte reads; ReadMultipleBytes(1)
            # returns stale data from the hub's buffer on fw 1.8.2.
            self.session.i2c_read_single_byte(
                self.dest, self.i2c_channel, self.address
            )
        else:
            self.session.i2c_read_multiple_bytes(
                self.dest, self.i2c_channel, self.address, num_bytes
            )
        # Phase 3: poll for completion.
        for _ in range(_MAX_POLLS):
            status, data = self.session.i2c_read_status_query(
                self.dest, self.i2c_channel
            )
            if status == 0:
                return data
            if status == _I2C_NACK:
                raise I2CError(
                    status,
                    self.i2c_channel,
                    self.address,
                    f"I2C NACK — device 0x{self.address:02X} not present or "
                    f"rejected read of register 0x{register:02X}",
                )
            if status == _I2C_TIMEOUT:
                raise I2CError(
                    status,
                    self.i2c_channel,
                    self.address,
                    f"I2C bus timeout reading register 0x{register:02X} from "
                    f"device 0x{self.address:02X} on channel {self.i2c_channel}",
                )
            if status != _I2C_IN_PROGRESS:
                raise I2CError(
                    status,
                    self.i2c_channel,
                    self.address,
                    f"Unexpected I2C status {status} reading register "
                    f"0x{register:02X} from device 0x{self.address:02X}",
                )
            time.sleep(_POLL_DELAY)
        raise RhspTimeoutError(
            f"I2C read poll timeout after {_MAX_POLLS} attempts "
            f"(channel={self.i2c_channel}, address=0x{self.address:02X}, "
            f"register=0x{register:02X})"
        )

    def __repr__(self) -> str:
        return (
            f"I2CDevice(dest={self.dest!r}, channel={self.i2c_channel!r}, "
            f"address=0x{self.address:02X})"
        )
