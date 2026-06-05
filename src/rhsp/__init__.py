"""rhsp — Pure-Python driver for the REV Robotics Hub serial protocol (RHSP / DEKA).

Foundation exports: error and enum types are available here.  Higher-level
objects (``connect``, ``Hub``, ``enumerate_hubs``) will be added by later
tickets as the rest of the library is implemented.
"""

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

__all__ = [
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
]
