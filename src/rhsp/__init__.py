"""rhsp — Pure-Python driver for the REV Robotics Hub serial protocol (RHSP / DEKA).

Public API surface:

- ``connect(port=None, ...) -> Hub``: open a hub connection.
- ``enumerate_hubs() -> list[str]``: list available REV Hub serial ports.
- ``Hub``: the connected hub object.
- ``Session``: the low-level transaction engine (advanced use).
- Error types: ``RhspError``, ``ProtocolError``, ``ChecksumError``,
  ``RhspTimeoutError``, ``NackError``.
- Enum types: ``MotorMode``, ``ZeroPowerBehavior``, ``NackCode``,
  ``ModuleStatusBits``, ``MotorStatusBits``, and more.
- Bulk dataclasses: ``BulkInputData``, ``ModuleStatus``.

Python ≥ 3.13 required.  API is snake_case throughout.
"""

from .discovery import connect, enumerate_hubs  # noqa: F401
from .hub import Hub  # noqa: F401
from .session import Session  # noqa: F401
from .errors import (  # noqa: F401
    ChecksumError,
    NackError,
    ProtocolError,
    RhspError,
    RhspTimeoutError,
)
from .enums import (  # noqa: F401
    ADCChannel,
    ClosedLoopMode,
    DIODirection,
    I2CSpeedCode,
    LEDColor,
    MotorMode,
    MotorStatusBits,
    ModuleStatusBits,
    NackCode,
    ZeroPowerBehavior,
)
from .devices.bulk import BulkInputData, ModuleStatus  # noqa: F401

__all__ = [
    # discovery / connection
    "connect",
    "enumerate_hubs",
    "Hub",
    "Session",
    # errors
    "RhspError",
    "ProtocolError",
    "ChecksumError",
    "RhspTimeoutError",
    "NackError",
    # enums
    "MotorMode",
    "ZeroPowerBehavior",
    "ClosedLoopMode",
    "DIODirection",
    "ADCChannel",
    "LEDColor",
    "I2CSpeedCode",
    "NackCode",
    "ModuleStatusBits",
    "MotorStatusBits",
    # bulk dataclasses
    "BulkInputData",
    "ModuleStatus",
]
