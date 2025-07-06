"""
I2C constants and functions for RHSP protocol.
"""
from . import messages as REVMsg
import time
from typing import Tuple, Any


class I2CConstants:
    """Constants for I2C communication and color sensor register addresses."""
    
    # I2C Command bits
    COMMAND_REGISTER_BIT = 128
    SINGLE_BYTE_BIT = 0
    MULTI_BYTE_BIT = 32
    
    # Color sensor configuration
    COLOR_SENSOR_ADDRESS = 57
    COLOR_SENSOR_ID = 96
    
    # Color sensor register addresses
    ENABLE_REGISTER = 0
    ATIME_REGISTER = 1
    WTIME_REGISTER = 3
    AILTL_REGISTER = 4
    AILTH_REGISTER = 5
    AIHTL_REGISTER = 6
    AIHTH_REGISTER = 7
    PILTL_REGISTER = 8
    PILTH_REGISTER = 9
    PIHTL_REGISTER = 10
    PIHTH_REGISTER = 11
    PERS_REGISTER = 12
    CONFIG_REGISTER = 13
    PPULSE_REGISTER = 14
    CONTROL_REGISTER = 15
    REVISION_REGISTER = 17
    ID_REGISTER = 18
    STATUS_REGISTER = 19
    
    # Color data registers
    CDATA_REGISTER = 20
    CDATAH_REGISTER = 21
    RDATA_REGISTER = 22
    RDATAH_REGISTER = 23
    GDATA_REGISTER = 24
    GDATAH_REGISTER = 25
    BDATA_REGISTER = 26
    BDATAH_REGISTER = 27
    PDATA_REGISTER = 28
    PDATAH_REGISTER = 29




def i2cWriteSingleByte(commObj: Any, destination: int, i2cChannel: int, slaveAddress: int, byteToWrite: int) -> None:
    """Write a single byte to an I2C device."""
    i2cWriteSingleByteMsg = REVMsg.I2CWriteSingleByte()
    i2cWriteSingleByteMsg.payload.i2cChannel = i2cChannel
    i2cWriteSingleByteMsg.payload.slaveAddress = slaveAddress
    i2cWriteSingleByteMsg.payload.byteToWrite = byteToWrite
    commObj.sendAndReceive(i2cWriteSingleByteMsg, destination)


def i2cWriteMultipleBytes(commObj: Any, destination: int, i2cChannel: int, slaveAddress: int, numBytes: int, bytesToWrite: int) -> None:
    """Write multiple bytes to an I2C device."""
    i2cWriteMultipleBytesMsg = REVMsg.I2CWriteMultipleBytes()
    i2cWriteMultipleBytesMsg.payload.i2cChannel = i2cChannel
    i2cWriteMultipleBytesMsg.payload.slaveAddress = slaveAddress
    i2cWriteMultipleBytesMsg.payload.numBytes = numBytes
    i2cWriteMultipleBytesMsg.payload.bytesToWrite = bytesToWrite
    commObj.sendAndReceive(i2cWriteMultipleBytesMsg, destination)


def i2cWriteStatusQuery(commObj: Any, destination: int, i2cChannel: int) -> Tuple[int, int]:
    """Query the status of an I2C write operation."""
    i2cWriteStatusQueryMsg = REVMsg.I2CWriteStatusQuery()
    i2cWriteStatusQueryMsg.payload.i2cChannel = i2cChannel
    packet = commObj.sendAndReceive(i2cWriteStatusQueryMsg, destination)
    return (
     packet.payload.i2cStatus, packet.payload.numBytes)


def i2cReadSingleByte(commObj: Any, destination: int, i2cChannel: int, slaveAddress: int) -> None:
    """Read a single byte from an I2C device."""
    i2cReadSingleByteMsg = REVMsg.I2CReadSingleByte()
    i2cReadSingleByteMsg.payload.i2cChannel = i2cChannel
    i2cReadSingleByteMsg.payload.slaveAddress = slaveAddress
    commObj.sendAndReceive(i2cReadSingleByteMsg, destination)


def i2cReadMultipleBytes(commObj: Any, destination: int, i2cChannel: int, slaveAddress: int, numBytes: int) -> None:
    """Read multiple bytes from an I2C device."""
    i2cReadMultipleBytesMsg = REVMsg.I2CReadMultipleBytes()
    i2cReadMultipleBytesMsg.payload.i2cChannel = i2cChannel
    i2cReadMultipleBytesMsg.payload.slaveAddress = slaveAddress
    i2cReadMultipleBytesMsg.payload.numBytes = numBytes
    commObj.sendAndReceive(i2cReadMultipleBytesMsg, destination)


def i2cReadStatusQuery(commObj: Any, destination: int, i2cChannel: int) -> Tuple[int, int, int]:
    """Query the status of an I2C read operation."""
    i2cReadStatusQueryMsg = REVMsg.I2CReadStatusQuery()
    i2cReadStatusQueryMsg.payload.i2cChannel = i2cChannel
    packet = commObj.sendAndReceive(i2cReadStatusQueryMsg, destination)
    return (
     packet.payload.i2cStatus, packet.payload.byteRead, packet.payload.payloadBytes)


def i2cConfigureChannel(commObj: Any, destination: int, i2cChannel: int, speedCode: int) -> None:
    """Configure an I2C channel with the specified speed."""
    i2cConfigureChannelMsg = REVMsg.I2CConfigureChannel()
    i2cConfigureChannelMsg.payload.i2cChannel = i2cChannel
    i2cConfigureChannelMsg.payload.speedCode = speedCode
    commObj.sendAndReceive(i2cConfigureChannelMsg, destination)


def i2cConfigureQuery(commObj: Any, destination: int, i2cChannel: int) -> int:
    """Query the configuration of an I2C channel."""
    i2cConfigureQueryMsg = REVMsg.I2CConfigureQuery()
    i2cConfigureQueryMsg.payload.i2cChannel = i2cChannel
    packet = commObj.sendAndReceive(i2cConfigureQueryMsg, destination)
    return packet.payload.speedCode


def i2cBlockReadConfig(commObj: Any, destination: int, i2cChannel: int, address: int, startRegister: int, numberOfBytes: int, readInterval_ms: int) -> None:
    """Configure block read for an I2C device."""
    i2cBlockReadConfigMsg = REVMsg.I2CBlockReadConfig()
    i2cBlockReadConfigMsg.channel = i2cChannel
    i2cBlockReadConfigMsg.address = address
    i2cBlockReadConfigMsg.startRegister = startRegister
    i2cBlockReadConfigMsg.numberOfBytes = numberOfBytes
    i2cBlockReadConfigMsg.readInterval_ms = readInterval_ms
    commObj.sendAndReceive(i2cBlockReadConfigMsg, destination)


def i2cBlockReadQuery(commObj: Any, destination: int) -> Any:
    """Query the results of a block read operation."""
    i2cBlockReadQueryMsg = REVMsg.I2CBlockReadQuery()
    packet = commObj.sendAndReceive(i2cBlockReadQueryMsg, destination)
    return packet


def imuBlockReadConfig(commObj: Any, destination: int, startRegister: int, numberOfBytes: int, readInterval_ms: int) -> None:
    """Configure block read for the IMU."""
    imuBlockReadConfigMsg = REVMsg.IMUBlockReadConfig()
    imuBlockReadConfigMsg.startRegister = startRegister
    imuBlockReadConfigMsg.numberOfBytes = numberOfBytes
    imuBlockReadConfigMsg.readInterval_ms = readInterval_ms
    commObj.sendAndReceive(imuBlockReadConfigMsg, destination)


def imuBlockReadQuery(commObj: Any, destination: int) -> Any:
    """Query the results of an IMU block read operation."""
    imuBlockReadQueryMsg = REVMsg.IMUBlockReadQuery()
    packet = commObj.sendAndReceive(imuBlockReadQueryMsg, destination)
    return packet

