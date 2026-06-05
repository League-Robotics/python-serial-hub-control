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
