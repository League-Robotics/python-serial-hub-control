"""I2CChannel and I2CDevice — typed wrappers for RHSP I2C commands.

P7-a fix: ``I2CChannel.configure(speed)`` passes ``self.session`` correctly
(the vendor implementation dropped the session reference).

``I2CDevice.read_register()`` implements the §5.8 two-phase poll protocol:
write the register address, then poll ``i2c_read_status_query`` until the
hub returns data (up to 5 polls × 1 ms).
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from rhsp.errors import RhspTimeoutError

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["I2CChannel", "I2CDevice"]

# I2C status code that means "still in progress — poll again" (NACK-41).
_I2C_IN_PROGRESS: int = 41

# Maximum number of read-status polls before raising RhspTimeoutError.
_MAX_POLLS: int = 5

# Delay between polls (seconds).
_POLL_DELAY: float = 0.001


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

    def configure(self, speed_code: int) -> None:
        """Configure the I2C bus speed for this channel.

        Parameters:
            speed_code: Speed code (see :class:`~rhsp.enums.I2CSpeedCode`).

        Note:
            P7-a fix: ``self.session`` is passed correctly; the vendor
            implementation omitted the session reference.
        """
        self.session.i2c_configure_channel(self.address, self.channel, speed_code)

    def device(self, address: int) -> "I2CDevice":
        """Create an :class:`I2CDevice` for an I2C peripheral at *address*.

        Parameters:
            address: 7-bit I2C device address.

        Returns:
            An :class:`I2CDevice` bound to this channel.
        """
        return I2CDevice(self.session, self.channel, self.address, address)

    def __repr__(self) -> str:
        return f"I2CChannel(address={self.address!r}, channel={self.channel!r})"


class I2CDevice:
    """Typed wrapper for a single I2C peripheral device.

    Provides register-level read/write operations over the RHSP I2C
    transaction protocol (§5.8: write then poll for data).

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
        ``I2CWriteMultipleBytes`` transaction.

        Parameters:
            register: Register address (0–255).
            data:     Bytes to write.
        """
        payload = bytes([register]) + bytes(data)
        self.session.i2c_write_multiple_bytes(
            self.dest, self.i2c_channel, self.address, payload
        )

    def read_register(self, register: int, num_bytes: int) -> bytes:
        """Read bytes from a device register (§5.8 two-phase poll).

        Phase 1: Issue ``I2CWriteMultipleBytes`` with the register byte.
        Phase 2: Issue ``I2CReadMultipleBytes`` to request *num_bytes*.
        Phase 3: Poll ``I2CReadStatusQuery`` until the hub reports data
                 (up to :data:`_MAX_POLLS` attempts × :data:`_POLL_DELAY` s).

        Parameters:
            register:  Register address (0–255).
            num_bytes: Number of bytes to read.

        Returns:
            The bytes read from the device.

        Raises:
            RhspTimeoutError: If the hub does not return data within the
                poll limit.
        """
        # Phase 1: write the register pointer.
        self.session.i2c_write_multiple_bytes(
            self.dest, self.i2c_channel, self.address, bytes([register])
        )
        # Phase 2: initiate the read.
        self.session.i2c_read_multiple_bytes(
            self.dest, self.i2c_channel, self.address, num_bytes
        )
        # Phase 3: poll for completion.
        for _ in range(_MAX_POLLS):
            status, data = self.session.i2c_read_status_query(
                self.dest, self.i2c_channel
            )
            if status != _I2C_IN_PROGRESS:
                return data
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
