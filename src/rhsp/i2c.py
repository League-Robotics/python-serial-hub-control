
from rhsp import Client
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.color import ColorSensor

from rhsp.color import ColorSensorV3
from rhsp.distance import Distance2m
from rhsp.imu import IMU
from rhsp.internal.i2c import (i2cBlockReadConfig, i2cBlockReadQuery, i2cConfigureChannel, 
                               i2cConfigureQuery, i2cReadMultipleBytes, i2cReadSingleByte, 
                               i2cReadStatusQuery, i2cWriteMultipleBytes, i2cWriteSingleByte)



class I2CDevice:

    def __init__(self, client: Client, channel, destinationModule, address):
        self.client = client
        self.channel = channel
        self.destinationModule = destinationModule
        self.address = address
        self.type = 'Generic'

    def setType(self, type):
        self.type = type

    def getType(self):
        return self.type

    def setChannel(self, channel):
        self.channel = channel

    def getChannel(self):
        return self.channel

    def setDestination(self, destinationModule):
        self.destinationModule = destinationModule

    def getDestination(self):
        return self.destinationModule

    def setAddress(self, address):
        self.address = address

    def getAddress(self):
        return self.address

    def writeByte(self, byteToWrite):
        i2cWriteSingleByte(self.client, self.destinationModule, self.channel, self.address, byteToWrite)

    def writeMultipleBytes(self, numBytes, bytesToWrite):
        i2cWriteMultipleBytes(self.client, self.destinationModule, self.channel, self.address, numBytes, bytesToWrite)

    def readByte(self):
        i2cReadSingleByte(self.client, self.destinationModule, self.channel, self.address)
        return int(i2cReadStatusQuery(self.client, self.destinationModule, self.channel)[2]) & 255

    def readMultipleBytes(self, numBytes):
        i2cReadMultipleBytes(self.client, self.destinationModule, self.channel, self.address, numBytes)
        byteMask = '0x'
        for i in range(0, numBytes):
            byteMask += 'FF'

        return int(i2cReadStatusQuery(self.client, self.destinationModule, self.channel)[2]) & int(byteMask, 16)

    def setBlockReadConfig(self, startRegister, numberOfBytes, readInterval_ms):
        i2cBlockReadConfig(self.client, self.destinationModule, self.channel, self.address, startRegister, numberOfBytes, readInterval_ms)

    def getBlockReadConfig(self):
        return i2cBlockReadQuery(self.client, self.destinationModule)


class I2CChannel:

    def __init__(self, client: Client, channel, destinationModule):
        self.client = client
        self.channel = channel
        self.destinationModule = destinationModule
        self.devices = {}

    def setChannel(self, channel):
        self.channel = channel

    def getChannel(self):
        return self.channel

    def setDestination(self, destinationModule):
        self.destinationModule = destinationModule

    def getDestination(self):
        return self.destinationModule

    def addDevice(self, address, name):
        self.devices[name] = I2CDevice(self.client, self.channel, self.destinationModule, address)

    def addColorSensor(self, name='color') -> 'ColorSensor':
        from rhsp.color import ColorSensor
        self.devices[name] = ColorSensor(self.client, self.channel, self.destinationModule)
        return self.devices[name]

    def addColorSensorV3(self, name='colorv3') -> 'ColorSensorV3':
        from rhsp.color import ColorSensorV3
        self.devices[name] = ColorSensorV3(self.client, self.channel, self.destinationModule)
        return self.devices[name]

    def addIMU(self, name='imu') -> 'IMU':
        from rhsp.imu import IMU
        self.devices[name] = IMU(self.client, self.channel, self.destinationModule)

    def addDistanceSensor(self, name='distance') -> 'Distance2m':
        from rhsp.distance import Distance2m 
        self.devices[name] = Distance2m(self.client, self.channel, self.destinationModule, False)
        return self.devices[name]

    def addI2CDevice(self, name, device):
        self.devices[name] = device

    def getDevices(self):
        return self.devices

    def setSpeed(self, speedCode):
        i2cConfigureChannel(self.destinationModule, self.channel, speedCode)
        return i2cConfigureQuery(self.destinationModule, self.channel)