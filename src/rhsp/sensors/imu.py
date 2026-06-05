"""IMU driver — BNO055 via the hub's internal IMU bus.

Unlike the I2C-attached color and distance sensors, the BNO055 is connected
directly to the REV Control Hub's internal sensor bus.  The hub handles
autonomous polling; the driver configures the polling with
``IMUBlockReadConfig`` and retrieves results from the hub's bulk-data buffer.

The IMU does NOT use an :class:`~rhsp.devices.i2c.I2CDevice`; it accepts a
:class:`~rhsp.session.Session` and a destination module address.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session
    from rhsp.devices.bulk import BulkInputData

__all__ = ["IMU"]

# BNO055 register where the 10-byte orientation block starts (EUL_H_LSB).
_IMU_START_REGISTER: int = 0x1A
_IMU_BLOCK_NUM_BYTES: int = 10
_IMU_READ_INTERVAL_MS: int = 10


class IMU:
    """Driver for the BNO055 IMU on the hub's internal sensor bus.

    Configures hub-side autonomous polling via ``IMUBlockReadConfig`` in
    ``__init__``.  Call :meth:`read_imu_block` to obtain the latest 10-byte
    orientation block from the hub's bulk input data buffer.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    dest:
        Hub module address on the RS-485 bus.
    """

    def __init__(self, session: "Session", dest: int) -> None:
        self._session = session
        self._dest = dest
        self._configure()

    def _configure(self) -> None:
        """Send IMUBlockReadConfig to start hub-side autonomous polling."""
        self._session.transaction(
            "IMUBlockReadConfig",
            self._dest,
            startRegister=_IMU_START_REGISTER,
            numberOfBytes=_IMU_BLOCK_NUM_BYTES,
            readInterval_ms=_IMU_READ_INTERVAL_MS,
        )

    def read_imu_block(self) -> bytes:
        """Read the latest 10-byte IMU block from the hub's bulk data buffer.

        Calls :meth:`~rhsp.session.Session.get_bulk_input_data` and extracts
        the ``imu_block`` field.

        Returns
        -------
        bytes
            10 bytes of raw IMU orientation data starting at BNO055 register
            ``EUL_H_LSB`` (0x1A).
        """
        bulk: "BulkInputData" = self._session.get_bulk_input_data(self._dest)
        return bulk.imu_block

    def __repr__(self) -> str:
        return f"IMU(dest={self._dest!r})"
