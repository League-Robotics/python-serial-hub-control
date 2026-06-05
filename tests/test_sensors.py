"""Hardware-free tests for sensor drivers (ticket 012).

All tests use FakeHub over LoopbackTransport — no real hardware required.

Coverage:
- registers.OSC_CALIBRATE_VAL == 0xF8  (P7-f)
- registers: APDS-9960 constants present
- ColorSensor.__init__ emits ENABLE / ATIME / PPULSE writes then ID read
- ColorSensor.__init__ raises ProtocolError on wrong device ID
- ColorSensor.read_color returns (red, green, blue, clear) decoded from 8 bytes
- Distance2m.__init__ emits full VL53L0X init sequence including OSC_CALIBRATE_VAL
- Distance2m.read_mm polls interrupt status, reads range, clears interrupt
- IMU.__init__ sends IMUBlockReadConfig with numberOfBytes=10
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Any

import pytest

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
from rhsp.codec import encode_payload
from rhsp.devices.i2c import I2CDevice
from rhsp.errors import ProtocolError
from rhsp.framing import build_frame, RawPacket
from rhsp.session import Session
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# DEKA base — same as all other device tests
# ---------------------------------------------------------------------------
_DEKA_BASE = 0x1000

# I2CWriteMultipleBytes = DEKA 0x26
_I2C_WRITE_TYPE = _DEKA_BASE + 0x26
# I2CReadMultipleBytes = DEKA 0x28
_I2C_READ_TYPE = _DEKA_BASE + 0x28
# I2CReadStatusQuery = DEKA 0x29
_I2C_READ_STATUS_TYPE = _DEKA_BASE + 0x29
# IMUBlockReadConfig = DEKA 0x35
_IMU_BLOCK_READ_CONFIG_TYPE = _DEKA_BASE + 0x35


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_session(timeout: float = 0.5) -> tuple[LoopbackTransport, FakeHub, Session]:
    transport = LoopbackTransport()
    hub = FakeHub(transport)
    session = Session(transport, timeout=timeout)
    return transport, hub, session


def _make_i2c_device(
    channel: int = 1, address: int = 0x39
) -> tuple[LoopbackTransport, FakeHub, Session, IFakeDevice]:
    """Return (transport, hub, session, device) with an I2CDevice at *address*."""
    transport, hub, session = _make_session()
    dev = I2CDevice(session, i2c_channel=channel, dest=1, address=address)
    return transport, hub, session, dev


class IFakeDevice:
    """Type alias placeholder — not used at runtime, just for readability."""


def _write_reqs(hub: FakeHub) -> list[RawPacket]:
    return [r for r in hub.requests if r.packet_type == _I2C_WRITE_TYPE]


def _read_reqs(hub: FakeHub) -> list[RawPacket]:
    return [r for r in hub.requests if r.packet_type == _I2C_READ_TYPE]


def _first_byte_of_write(pkt: RawPacket) -> int:
    """Extract the first data byte from an I2CWriteMultipleBytes payload.

    Payload layout:
      byte 0: i2c_channel
      byte 1: slave_address
      byte 2: num_bytes
      byte 3..: bytesToWrite
    The first byte of bytesToWrite[0] is the register address; bytesToWrite[1]
    is the data value (for a register write).
    """
    return pkt.payload[3]


def _bytes_of_write(pkt: RawPacket) -> bytes:
    """Return the full bytesToWrite portion of an I2CWriteMultipleBytes payload."""
    num_bytes = pkt.payload[2]
    return bytes(pkt.payload[3 : 3 + num_bytes])


def _setup_read_data(
    hub: FakeHub,
    data: bytes,
    *,
    status_code: int = 0,
) -> None:
    """Pre-configure FakeHub to return *data* on the next I2CReadStatusQuery."""
    hub.set_rsp(
        "I2CReadStatusQuery",
        i2cStatus=status_code,
        byteRead=len(data),
        payloadBytes=data,
    )


# ===========================================================================
# Register constants
# ===========================================================================


class TestRegisterConstants:
    """Verify all register constants required by the ticket."""

    def test_osc_calibrate_val_is_0xf8(self) -> None:
        from rhsp.sensors.registers import OSC_CALIBRATE_VAL

        assert OSC_CALIBRATE_VAL == 0xF8

    def test_apds9960_constants_present(self) -> None:
        from rhsp.sensors.registers import (
            COMMAND_BIT,
            MULTI_BYTE_BIT,
            APDS9960_ENABLE,
            APDS9960_ATIME,
            APDS9960_PPULSE,
            APDS9960_ID,
            APDS9960_CDATAL,
            APDS9960_RDATAL,
            APDS9960_GDATAL,
            APDS9960_BDATAL,
            APDS9960_DEVICE_ID,
        )

        assert COMMAND_BIT == 0x80
        assert MULTI_BYTE_BIT == 0x20
        assert APDS9960_DEVICE_ID == 0x60

    def test_vl53l0x_osc_calibrate_val_in_registers(self) -> None:
        """OSC_CALIBRATE_VAL must be importable from registers module."""
        from rhsp.sensors.registers import OSC_CALIBRATE_VAL

        assert OSC_CALIBRATE_VAL == 0xF8

    def test_bno055_constants_present(self) -> None:
        from rhsp.sensors.registers import (
            BNO055_ADDRESS,
            BNO055_OPR_MODE,
            BNO055_PWR_MODE,
            BNO055_SYS_TRIGGER,
            BNO055_CONFIGMODE,
            BNO055_IMUMODE,
            BNO055_NORMAL,
        )

        assert BNO055_ADDRESS == 0x28


# ===========================================================================
# ColorSensor (APDS-9960) — FakeHub sequence tests
# ===========================================================================


class TestColorSensor:
    """ColorSensor init emits ENABLE / ATIME / PPULSE writes then ID read."""

    def _make_color_device(
        self,
    ) -> tuple[LoopbackTransport, FakeHub, Session, Any]:
        transport, hub, session = _make_session()
        # I2C channel 1, APDS-9960 at address 0x39
        dev = I2CDevice(session, i2c_channel=1, dest=1, address=0x39)
        return transport, hub, session, dev

    def test_init_emits_enable_atime_ppulse_then_id_read(self) -> None:
        """ColorSensor.__init__ writes ENABLE, ATIME, PPULSE, then reads ID."""
        from rhsp.sensors.color import ColorSensor
        from rhsp.sensors.registers import (
            COMMAND_BIT,
            MULTI_BYTE_BIT,
            APDS9960_ENABLE,
            APDS9960_ATIME,
            APDS9960_PPULSE,
            APDS9960_ID,
            APDS9960_DEVICE_ID,
        )

        transport, hub, session, dev = self._make_color_device()
        # Pre-load correct device ID for the read
        _setup_read_data(hub, bytes([APDS9960_DEVICE_ID]))
        hub.run_in_thread()
        try:
            sensor = ColorSensor(dev)
        finally:
            hub.stop()

        writes = _write_reqs(hub)
        reads = _read_reqs(hub)

        # Expect at least 3 register writes (ENABLE, ATIME, PPULSE)
        # plus 1 register-pointer write before the read
        assert len(writes) >= 3, f"Expected >= 3 writes, got {len(writes)}"
        assert len(reads) >= 1, f"Expected >= 1 read, got {len(reads)}"

        # Extract register bytes from each write (payload[3] = first data byte)
        register_bytes = [_first_byte_of_write(w) for w in writes]

        expected_enable_reg = COMMAND_BIT | APDS9960_ENABLE
        expected_atime_reg = COMMAND_BIT | APDS9960_ATIME
        expected_ppulse_reg = COMMAND_BIT | APDS9960_PPULSE

        assert expected_enable_reg in register_bytes, (
            f"ENABLE register 0x{expected_enable_reg:02X} not in writes: {[hex(b) for b in register_bytes]}"
        )
        assert expected_atime_reg in register_bytes, (
            f"ATIME register 0x{expected_atime_reg:02X} not in writes"
        )
        assert expected_ppulse_reg in register_bytes, (
            f"PPULSE register 0x{expected_ppulse_reg:02X} not in writes"
        )

        # ENABLE write must come before ATIME write
        enable_idx = next(
            i for i, w in enumerate(writes)
            if _first_byte_of_write(w) == expected_enable_reg
        )
        atime_idx = next(
            i for i, w in enumerate(writes)
            if _first_byte_of_write(w) == expected_atime_reg
        )
        ppulse_idx = next(
            i for i, w in enumerate(writes)
            if _first_byte_of_write(w) == expected_ppulse_reg
        )
        assert enable_idx < atime_idx, "ENABLE must be written before ATIME"
        assert atime_idx < ppulse_idx, "ATIME must be written before PPULSE"

        # ID register-pointer write must occur after PPULSE write
        expected_id_reg = COMMAND_BIT | MULTI_BYTE_BIT | APDS9960_ID
        id_write_idx = next(
            (i for i, w in enumerate(writes)
             if _first_byte_of_write(w) == expected_id_reg),
            None,
        )
        assert id_write_idx is not None, (
            f"ID register pointer 0x{expected_id_reg:02X} not found in writes"
        )
        assert id_write_idx > ppulse_idx, "ID read must come after PPULSE write"

    def test_init_raises_protocol_error_on_wrong_device_id(self) -> None:
        """ColorSensor.__init__ raises ProtocolError if device ID != 0x60."""
        from rhsp.sensors.color import ColorSensor

        transport, hub, session, dev = self._make_color_device()
        # Return wrong device ID (0xAB instead of 0x60)
        _setup_read_data(hub, bytes([0xAB]))
        hub.run_in_thread()
        try:
            with pytest.raises(ProtocolError, match="unexpected color sensor id"):
                ColorSensor(dev)
        finally:
            hub.stop()

    def test_read_color_returns_rgbc_tuple(self) -> None:
        """ColorSensor.read_color() returns (red, green, blue, clear) from 8 LE bytes."""
        from rhsp.sensors.color import ColorSensor
        from rhsp.sensors.registers import APDS9960_DEVICE_ID

        transport, hub, session, dev = self._make_color_device()

        # First call during __init__ returns device ID
        # We need to set up the sequence: init ID read first, then color read
        # Use a counter-based approach
        call_count = [0]
        original_handle = hub._handle

        # Pre-encoded 8-byte color data: C=100, R=200, G=150, B=75 (LE 16-bit each)
        color_data = struct.pack("<HHHH", 100, 200, 150, 75)  # C, R, G, B

        def _patched_handle(pkt: Any) -> None:
            from rhsp.catalogue import COMMANDS as CMDS

            cmd_name_map = {cmd.id: name for name, cmd in CMDS.items()}
            cname = cmd_name_map.get(pkt.packet_type)
            if cname == "I2CReadStatusQuery":
                call_count[0] += 1
                if call_count[0] == 1:
                    # First read: device ID
                    hub._rsp_config["I2CReadStatusQuery"] = {
                        "i2cStatus": 0,
                        "byteRead": 1,
                        "payloadBytes": bytes([APDS9960_DEVICE_ID]),
                    }
                else:
                    # Subsequent reads: color data
                    hub._rsp_config["I2CReadStatusQuery"] = {
                        "i2cStatus": 0,
                        "byteRead": 8,
                        "payloadBytes": color_data,
                    }
            original_handle(pkt)

        hub._handle = _patched_handle
        hub.run_in_thread()
        try:
            sensor = ColorSensor(dev)
            result = sensor.read_color()
        finally:
            hub.stop()

        # result = (red, green, blue, clear)
        # From struct.pack("<HHHH", 100, 200, 150, 75): C=100, R=200, G=150, B=75
        assert isinstance(result, tuple)
        assert len(result) == 4
        red, green, blue, clear = result
        assert red == 200
        assert green == 150
        assert blue == 75
        assert clear == 100


# ===========================================================================
# Distance2m (VL53L0X) — FakeHub sequence tests
# ===========================================================================


class TestDistance2m:
    """Distance2m init emits full VL53L0X sequence including OSC_CALIBRATE_VAL."""

    def _make_distance_device(
        self,
    ) -> tuple[LoopbackTransport, FakeHub, Session, Any]:
        transport, hub, session = _make_session()
        # I2C channel 0, VL53L0X at address 0x29
        dev = I2CDevice(session, i2c_channel=0, dest=1, address=0x29)
        return transport, hub, session, dev

    def _setup_full_init_responses(self, hub: FakeHub) -> None:
        """Configure FakeHub to respond to all reads in the VL53L0X init sequence.

        Read sequence (presence check first, then ``Distance2m.initialize()``):
          1  - presence check: _dev.read_register(0xC0, 1) → 0xEE (model id)
          In initialize():
          2  - _read1(0x91)                 stop_variable
          3  - _read1(MSRC_CONFIG_CONTROL)  | 0x12
          In _get_spad_info():
          4  - _read1(0x83)                 | 4   (first)
          5  - _read1(0x92)                 spad_count
          6  - _read1(0x83)                 & ~4  (second)
          Back in initialize():
          7  - read_register(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6)  6-byte SPAD map
          8  - _read1(GPIO_HV_MUX_ACTIVE_HIGH)                      & ~0x10
        """
        read_count = [0]
        original_handle = hub._handle

        def _patched_handle(pkt: Any) -> None:
            from rhsp.catalogue import COMMANDS as CMDS

            cmd_name_map = {cmd.id: name for name, cmd in CMDS.items()}
            cname = cmd_name_map.get(pkt.packet_type)
            if cname == "I2CReadStatusQuery":
                read_count[0] += 1
                n = read_count[0]
                if n == 1:
                    # Presence check: register 0xC0 → VL53L0X model id 0xEE
                    data = bytes([0xEE])
                elif n == 5:
                    # spad_count = 5, non-aperture type (bit7 = 0)
                    data = bytes([0x05])
                elif n == 7:
                    # SPAD reference enable map — 6 bytes
                    data = bytes([0xFF, 0x00, 0x00, 0x00, 0x00, 0x00])
                elif n == 8:
                    # GPIO_HV_MUX_ACTIVE_HIGH — return 0x10 so & ~0x10 = 0x00
                    data = bytes([0x10])
                else:
                    data = bytes([0x00])
                hub._rsp_config["I2CReadStatusQuery"] = {
                    "i2cStatus": 0,
                    "byteRead": len(data),
                    "payloadBytes": data,
                }
            original_handle(pkt)

        hub._handle = _patched_handle

    def test_init_emits_osc_calibrate_val_write(self) -> None:
        """Distance2m.__init__ writes OSC_CALIBRATE_VAL (0xF8) to register 0x01."""
        from rhsp.sensors.distance import Distance2m
        from rhsp.sensors.registers import OSC_CALIBRATE_VAL

        transport, hub, session, dev = self._make_distance_device()
        self._setup_full_init_responses(hub)
        hub.run_in_thread()
        try:
            sensor = Distance2m(dev)
        finally:
            hub.stop()

        writes = _write_reqs(hub)

        # Look for a write where register=0x01 and data=OSC_CALIBRATE_VAL
        osc_writes = []
        for w in writes:
            data = _bytes_of_write(w)
            if len(data) >= 2 and data[0] == 0x01 and data[1] == OSC_CALIBRATE_VAL:
                osc_writes.append(w)

        assert len(osc_writes) >= 1, (
            f"Expected at least one write of OSC_CALIBRATE_VAL=0x{OSC_CALIBRATE_VAL:02X} "
            f"to register 0x01; writes seen: "
            + ", ".join(f"[{' '.join(f'{b:02X}' for b in _bytes_of_write(w))}]" for w in writes[:10])
        )

    def test_init_emits_many_register_writes(self) -> None:
        """Distance2m.__init__ emits the lengthy ST tuning register sequence."""
        from rhsp.sensors.distance import Distance2m

        transport, hub, session, dev = self._make_distance_device()
        self._setup_full_init_responses(hub)
        hub.run_in_thread()
        try:
            sensor = Distance2m(dev)
        finally:
            hub.stop()

        writes = _write_reqs(hub)
        # The VL53L0X init sequence has 70+ register writes
        assert len(writes) > 50, (
            f"Expected > 50 register writes in VL53L0X init, got {len(writes)}"
        )

    def test_read_mm_polls_interrupt_reads_range_clears(self) -> None:
        """Distance2m.read_mm() issues interrupt poll, range read, interrupt clear."""
        from rhsp.sensors.distance import Distance2m
        from rhsp.sensors.registers import SYSTEM_INTERRUPT_CLEAR

        transport, hub, session, dev = self._make_distance_device()

        read_count = [0]
        original_handle = hub._handle

        # Init has 8 reads (see _setup_full_init_responses for breakdown).
        # read_mm then issues:
        #   read 9:  RESULT_INTERRUPT_STATUS → 0x07 (ready)
        #   read 10: RESULT_RANGE_STATUS+10  → 450 mm big-endian (0x01C2)

        def _patched_handle(pkt: Any) -> None:
            from rhsp.catalogue import COMMANDS as CMDS

            cmd_name_map = {cmd.id: name for name, cmd in CMDS.items()}
            cname = cmd_name_map.get(pkt.packet_type)
            if cname == "I2CReadStatusQuery":
                read_count[0] += 1
                n = read_count[0]
                if n == 1:
                    data = bytes([0xEE])  # presence check: model id
                elif n == 5:
                    data = bytes([0x05])  # spad_count
                elif n == 7:
                    data = bytes([0xFF, 0x00, 0x00, 0x00, 0x00, 0x00])  # SPAD map
                elif n == 8:
                    data = bytes([0x10])  # GPIO_HV_MUX_ACTIVE_HIGH
                elif n == 9:
                    # RESULT_INTERRUPT_STATUS = 0x07 (new sample ready)
                    data = bytes([0x07])
                elif n == 10:
                    # Range result at RESULT_RANGE_STATUS+10 = 450 mm big-endian
                    data = bytes([0x01, 0xC2])  # 0x01C2 = 450
                else:
                    data = bytes([0x00])
                hub._rsp_config["I2CReadStatusQuery"] = {
                    "i2cStatus": 0,
                    "byteRead": len(data),
                    "payloadBytes": data,
                }
            original_handle(pkt)

        hub._handle = _patched_handle
        hub.run_in_thread()
        try:
            sensor = Distance2m(dev)
            # Override timeout to prevent hanging
            sensor.set_timeout(500)
            range_mm = sensor.read_mm()
        finally:
            hub.stop()

        # Verify range value was decoded correctly
        assert range_mm == 450, f"Expected 450 mm, got {range_mm}"

        # Verify interrupt clear was issued after the range read
        writes_after = [
            w for w in _write_reqs(hub)
            if _bytes_of_write(w)[:2] == bytes([SYSTEM_INTERRUPT_CLEAR, 0x01])
        ]
        assert len(writes_after) >= 1, "Expected SYSTEM_INTERRUPT_CLEAR write after range read"


# ===========================================================================
# IMU — FakeHub session tests
# ===========================================================================


class TestIMU:
    """IMU.__init__ sends IMUBlockReadConfig with numberOfBytes=10."""

    def test_init_sends_imu_block_read_config(self) -> None:
        """IMU.__init__ sends IMUBlockReadConfig with numberOfBytes=10."""
        from rhsp.sensors.imu import IMU

        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            imu = IMU(session, dest=1)
        finally:
            hub.stop()

        # Find IMUBlockReadConfig requests
        imu_config_reqs = [
            r for r in hub.requests if r.packet_type == _IMU_BLOCK_READ_CONFIG_TYPE
        ]
        assert len(imu_config_reqs) == 1, (
            f"Expected 1 IMUBlockReadConfig request, got {len(imu_config_reqs)}"
        )

        # Decode the payload and verify numberOfBytes=10
        req = imu_config_reqs[0]
        cmd = COMMANDS["IMUBlockReadConfig"]
        from rhsp.codec import decode_payload

        decoded = decode_payload(cmd.fields, req.payload)
        assert decoded["numberOfBytes"] == 10, (
            f"Expected numberOfBytes=10, got {decoded['numberOfBytes']}"
        )

    def test_init_read_interval_is_10ms(self) -> None:
        """IMU.__init__ configures readInterval_ms=10."""
        from rhsp.sensors.imu import IMU

        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            imu = IMU(session, dest=1)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests if r.packet_type == _IMU_BLOCK_READ_CONFIG_TYPE
        )
        # IMUBlockReadConfig payload: startRegister(1), numberOfBytes(1), readInterval_ms(1)
        # The last byte is readInterval_ms = 10
        assert req.payload[2] == 10, (
            f"Expected readInterval_ms=10 at payload byte 2, got {req.payload[2]}"
        )

    def test_read_imu_block_returns_10_bytes(self) -> None:
        """IMU.read_imu_block() returns 10 bytes from bulk imu_block field."""
        from rhsp.sensors.imu import IMU

        transport, hub, session = _make_session()

        # IMU init + bulk read sequence
        imu_data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A])

        # Build a 141-byte bulk payload (all zeros except imuBlock at offset 122).
        # imuBlock starts at offset 122 and is 10 bytes (from catalogue).
        rsp_id = COMMANDS["GetBulkInputData"].reply_id
        bulk_payload = bytearray(141)
        bulk_payload[122:132] = imu_data  # imuBlock offset

        hub.run_in_thread()
        try:
            imu = IMU(session, dest=1)

            # Inject bulk response
            expected_ref = session._msg_num

            def _inject() -> None:
                time.sleep(0.02)
                transport.inject(
                    build_frame(
                        dest=0, src=1, msg=0, ref=expected_ref,
                        ptype=rsp_id, payload=bytes(bulk_payload),
                    )
                )

            t = threading.Thread(target=_inject, daemon=True)
            t.start()
            result = imu.read_imu_block()
            t.join(timeout=1.0)
        finally:
            hub.stop()

        assert isinstance(result, (bytes, bytearray)), f"Expected bytes, got {type(result)}"
        assert len(result) == 10, f"Expected 10 bytes, got {len(result)}"
        assert bytes(result) == imu_data
