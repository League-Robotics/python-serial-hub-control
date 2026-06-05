"""
Motor control class for RSHP protocol.
"""
from . import adc, module

from typing import TYPE_CHECKING, List, Any, Tuple

if TYPE_CHECKING:
    from .client import Client
    from .module import Module

from .internal.motors import (
    Q16, MODE_CONSTANT_POWER, MODE_CONSTANT_VELOCITY, MODE_POSITION_TARGET, MODE_CONSTANT_CURRENT,
    BRAKE_AT_ZERO, FLOAT_AT_ZERO, VELOCITY_OFFSET, CURRENT_OFFSET,
    setMotorChannelMode, getMotorChannelMode, setMotorChannelEnable, getMotorChannelEnable,
    setMotorChannelCurrentAlertLevel, getMotorChannelCurrentAlertLevel, resetMotorEncoder,
    setMotorConstantPower, getMotorConstantPower, setMotorTargetVelocity, getMotorTargetVelocity,
    setMotorTargetPosition, getMotorTargetPosition, getMotorAtTarget, getMotorEncoderPosition,
    setMotorPIDCoefficients, getMotorPIDCoefficients, getBulkPIDData,
    setCurrentPIDCoefficients, setVelocityPIDCoefficients, setPositionPIDCoefficients,
    getCurrentPIDCoefficients, getVelocityPIDCoefficients, getPositionPIDCoefficients
)


class Motor:

    def __init__(self, module: "Module", client: "Client", channel, destinationModule: int):

        self.channel = channel

        # module is the module object, and destinationModule is the module address,
        # they are redundant but kept to reduce changes.
        self.module = module
        self.destinationModule = destinationModule
        self.client = client
        
        self.motorCurrent = adc.ADCPin(self.client, 8 + channel, self.destinationModule)

    def setDestination(self, destinationModule):
        self.destinationModule = destinationModule
        self.motorCurrent.setDestination(destinationModule)

    def getDestination(self):
        return self.destinationModule

    def setChannel(self, channel):
        self.channel = channel

    def getChannel(self):
        return self.channel

    def setMode(self, mode, zeroFloat):
        setMotorChannelMode(self.client, self.destinationModule, self.channel, mode, zeroFloat)

    def getMode(self):
        return getMotorChannelMode(self.client, self.destinationModule, self.channel)

    def enable(self):
        setMotorChannelEnable(self.client, self.destinationModule, self.channel, 1)

    def disable(self):
        setMotorChannelEnable(self.client, self.destinationModule, self.channel, 0)

    def isEnabled(self):
        return getMotorChannelEnable(self.client, self.destinationModule, self.channel)

    def setCurrentLimit(self, limit):
        setMotorChannelCurrentAlertLevel(self.client, self.destinationModule, self.channel, limit)

    def getCurrentLimit(self):
        return getMotorChannelCurrentAlertLevel(self.client, self.destinationModule, self.channel)

    def resetEncoder(self):
        resetMotorEncoder(self.client, self.destinationModule, self.channel)

    def setPower(self, powerLevel):
        setMotorConstantPower(self.client, self.destinationModule, self.channel, powerLevel)

    def getPower(self):
        return getMotorConstantPower(self.client, self.destinationModule, self.channel)

    def setTargetCurrent(self, current):
        self.setCurrentLimit(current)

    def getTargetCurrent(self):
        return self.getCurrentLimit()

    def setTargetVelocity(self, velocity):
        setMotorTargetVelocity(self.client, self.destinationModule, self.channel, velocity)

    def getTargetVelocity(self):
        return getMotorTargetVelocity(self.client, self.destinationModule, self.channel)

    def setTargetPosition(self, position, tolerance):
        setMotorTargetPosition(self.client, self.destinationModule, self.channel, position, tolerance)

    def getTargetPosition(self):
        return getMotorTargetPosition(self.client, self.destinationModule, self.channel)

    def isAtTarget(self):
        return getMotorAtTarget(self.client, self.destinationModule, self.channel)

    def getPosition(self):
        position = getMotorEncoderPosition(self.client, self.destinationModule, self.channel)
        return position

    def resetPosition(self):
        resetMotorEncoder(self.client, self.destinationModule, self.channel)

    def getVelocity(self):
       

        bulkData = self.module.getBulkMotorData()
       
        try:
            val = int(bulkData[self.channel + VELOCITY_OFFSET])
            bits = int(16)
            if val & 1 << bits - 1 != 0:
                val = val - (1 << bits)
            return val
        except Exception:
            print("Failed to get velocity", bulkData)
            return 0

    def getCurrent(self):
        return self.motorCurrent.getADC(0)

    def setCurrentPID(self, p, i, d):
        setCurrentPIDCoefficients(self.client, self.destinationModule, self.channel, p, i, d)

    def getCurrentPID(self, p, i, d):
        return getCurrentPIDCoefficients(self.client, self.destinationModule, self.channel)

    def setVelocityPID(self, p, i, d):
        setVelocityPIDCoefficients(self.client, self.destinationModule, self.channel, p, i, d)

    def getVelocityPID(self):
        return getVelocityPIDCoefficients(self.client, self.destinationModule, self.channel)

    def setPositionPID(self, p, i, d):
        setPositionPIDCoefficients(self.client, self.destinationModule, self.channel, p, i, d)

    def getPositionPID(self):
        return getPositionPIDCoefficients(self.client, self.destinationModule, self.channel)

    def getBulkPIDData(self):
        return getBulkPIDData(self.client, self.destinationModule, self.channel)

    def init(self):
        self.setMode(0, 1)
        self.setPower(0)
        self.enable()
