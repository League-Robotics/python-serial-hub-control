"""Exception hierarchy for the rhsp library.

All exceptions raised by the library are subclasses of RhspError. Import from
this module or from the top-level `rhsp` package.

No imports from any other rhsp module — this module is a foundation leaf.
"""

__all__ = [
    "RhspError",
    "ProtocolError",
    "ChecksumError",
    "RhspTimeoutError",
    "NackError",
    "I2CError",
]


class RhspError(Exception):
    """Base class for all rhsp library errors."""


class ProtocolError(RhspError):
    """Raised on framing or protocol violations (e.g. unexpected packet structure)."""


class ChecksumError(ProtocolError):
    """Raised when a received packet has an incorrect checksum."""


class RhspTimeoutError(RhspError):
    """Raised when a transaction does not receive a response within the timeout."""


class NackError(RhspError):
    """Raised when the hub rejects a command with a NACK packet.

    Attributes:
        code: The numeric NACK code from the hub.
        description: Human-readable description of the NACK code.
    """

    def __init__(self, code: int, description: str) -> None:
        self.code = code
        self.description = description
        super().__init__(f"NACK {code}: {description}")


class I2CError(RhspError):
    """Raised when an I2C operation fails at the bus level.

    Attributes:
        i2c_status: The ``i2cStatus`` code returned by the hub (e.g. 3 for
            NACK/absent device, 46 for bus timeout).
        channel: I2C channel index (0–3).
        address: 7-bit I2C device address.
    """

    def __init__(
        self,
        i2c_status: int,
        channel: int,
        address: int,
        message: str = "",
    ) -> None:
        self.i2c_status = i2c_status
        self.channel = channel
        self.address = address
        desc = message or f"I2C status {i2c_status}"
        super().__init__(
            f"{desc} (channel={channel}, address=0x{address:02X}, i2cStatus={i2c_status})"
        )
