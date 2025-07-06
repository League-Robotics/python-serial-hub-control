"""
Motor control class for RSHP protocol.
"""
from . import adc, module
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

    def __init__(self, commObj, channel, destinationModule):
        self.channel = channel
        self.destinationModule = destinationModule
        self.commObj = commObj
        self.motorCurrent = adc.ADCPin(self.commObj, 8 + channel, self.destinationModule)

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
        setMotorChannelMode(self.commObj, self.destinationModule, self.channel, mode, zeroFloat)

    def getMode(self):
        return getMotorChannelMode(self.commObj, self.destinationModule, self.channel)

    def enable(self):
        setMotorChannelEnable(self.commObj, self.destinationModule, self.channel, 1)

    def disable(self):
        setMotorChannelEnable(self.commObj, self.destinationModule, self.channel, 0)

    def isEnabled(self):
        return getMotorChannelEnable(self.commObj, self.destinationModule, self.channel)

    def setCurrentLimit(self, limit):
        setMotorChannelCurrentAlertLevel(self.commObj, self.destinationModule, self.channel, limit)

    def getCurrentLimit(self):
        return getMotorChannelCurrentAlertLevel(self.commObj, self.destinationModule, self.channel)

    def resetEncoder(self):
        resetMotorEncoder(self.commObj, self.destinationModule, self.channel)

    def setPower(self, powerLevel):
        setMotorConstantPower(self.commObj, self.destinationModule, self.channel, powerLevel)

    def getPower(self):
        return getMotorConstantPower(self.commObj, self.destinationModule, self.channel)

    def setTargetCurrent(self, current):
        self.setCurrentLimit(current)

    def getTargetCurrent(self):
        return self.getCurrentLimit()

    def setTargetVelocity(self, velocity):
        setMotorTargetVelocity(self.commObj, self.destinationModule, self.channel, velocity)

    def getTargetVelocity(self):
        return getMotorTargetVelocity(self.commObj, self.destinationModule, self.channel)

    def setTargetPosition(self, position, tolerance):
        setMotorTargetPosition(self.commObj, self.destinationModule, self.channel, position, tolerance)

    def getTargetPosition(self):
        return getMotorTargetPosition(self.commObj, self.destinationModule, self.channel)

    def isAtTarget(self):
        return getMotorAtTarget(self.commObj, self.destinationModule, self.channel)

    def getPosition(self):
        position = getMotorEncoderPosition(self.commObj, self.destinationModule, self.channel)
        return position

    def resetPosition(self):
        resetMotorEncoder(self.commObj, self.destinationModule, self.channel)

    def getVelocity(self):
        bulkData = module.getBulkInputData(self.commObj, self.destinationModule)
        val = int(bulkData[self.channel + VELOCITY_OFFSET])
        bits = int(16)
        if val & 1 << bits - 1 != 0:
            val = val - (1 << bits)
        return val

    def getCurrent(self):
        return self.motorCurrent.getADC(0)

    def setCurrentPID(self, p, i, d):
        setCurrentPIDCoefficients(self.commObj, self.destinationModule, self.channel, p, i, d)

    def getCurrentPID(self, p, i, d):
        return getCurrentPIDCoefficients(self.commObj, self.destinationModule, self.channel)

    def setVelocityPID(self, p, i, d):
        setVelocityPIDCoefficients(self.commObj, self.destinationModule, self.channel, p, i, d)

    def getVelocityPID(self):
        return getVelocityPIDCoefficients(self.commObj, self.destinationModule, self.channel)

    def setPositionPID(self, p, i, d):
        setPositionPIDCoefficients(self.commObj, self.destinationModule, self.channel, p, i, d)

    def getPositionPID(self):
        return getPositionPIDCoefficients(self.commObj, self.destinationModule, self.channel)

    def getBulkPIDData(self):
        return getBulkPIDData(self.commObj, self.destinationModule, self.channel)

    def init(self):
        self.setMode(0, 1)
        self.setPower(0)
        self.enable()
