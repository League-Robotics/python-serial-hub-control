"""
Internal motor control functions for RSHP protocol.
"""
from . import messages as REVMsg

Q16 = 65536.0
MODE_CONSTANT_POWER = 0
MODE_CONSTANT_VELOCITY = 1
MODE_POSITION_TARGET = 2
MODE_CONSTANT_CURRENT = 3
BRAKE_AT_ZERO = 0
FLOAT_AT_ZERO = 1
VELOCITY_OFFSET = 6
CURRENT_OFFSET = 8


def setMotorChannelMode(commObj, destination, motorChannel, motorMode, floatAtZero):
    setMotorChannelModeMsg = REVMsg.SetMotorChannelMode()
    setMotorChannelModeMsg.payload.motorChannel = motorChannel
    setMotorChannelModeMsg.payload.motorMode = motorMode
    setMotorChannelModeMsg.payload.floatAtZero = floatAtZero
    commObj.sendAndReceive(setMotorChannelModeMsg, destination)


def getMotorChannelMode(commObj, destination, motorChannel):
    getMotorChannelModeMsg = REVMsg.GetMotorChannelMode()
    getMotorChannelModeMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorChannelModeMsg, destination)
    return (
     packet.payload.motorChannelMode, packet.payload.floatAtZero)


def setMotorChannelEnable(commObj, destination, motorChannel, enabled):
    setMotorChannelEnableMsg = REVMsg.SetMotorChannelEnable()
    setMotorChannelEnableMsg.payload.motorChannel = motorChannel
    setMotorChannelEnableMsg.payload.enabled = enabled
    packet = commObj.sendAndReceive(setMotorChannelEnableMsg, destination)


def getMotorChannelEnable(commObj, destination, motorChannel):
    getMotorChannelEnableMsg = REVMsg.GetMotorChannelEnable()
    getMotorChannelEnableMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorChannelEnableMsg, destination)
    return packet.payload.enabled


def setMotorChannelCurrentAlertLevel(commObj, destination, motorChannel, currentLimit):
    setMotorChannelCurrentAlertLevelMsg = REVMsg.SetMotorChannelCurrentAlertLevel()
    setMotorChannelCurrentAlertLevelMsg.payload.motorChannel = motorChannel
    setMotorChannelCurrentAlertLevelMsg.payload.currentLimit = currentLimit
    commObj.sendAndReceive(setMotorChannelCurrentAlertLevelMsg, destination)


def getMotorChannelCurrentAlertLevel(commObj, destination, motorChannel):
    getMotorChannelCurrentAlertLevelMsg = REVMsg.GetMotorChannelCurrentAlertLevel()
    getMotorChannelCurrentAlertLevelMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorChannelCurrentAlertLevelMsg, destination)
    return packet.payload.currentLimit


def resetMotorEncoder(commObj, destination, motorChannel):
    resetMotorEncoderMsg = REVMsg.ResetMotorEncoder()
    resetMotorEncoderMsg.payload.motorChannel = motorChannel
    commObj.sendAndReceive(resetMotorEncoderMsg, destination)


def setMotorConstantPower(commObj, destination, motorChannel, powerLevel):
    setMotorConstantPowerMsg = REVMsg.SetMotorConstantPower()
    setMotorConstantPowerMsg.payload.motorChannel = motorChannel
    setMotorConstantPowerMsg.payload.powerLevel = powerLevel
    commObj.sendAndReceive(setMotorConstantPowerMsg, destination)


def getMotorConstantPower(commObj, destination, motorChannel):
    getMotorConstantPowerMsg = REVMsg.GetMotorConstantPower()
    getMotorConstantPowerMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorConstantPowerMsg, destination)
    return packet.payload.powerLevel


def setMotorTargetVelocity(commObj, destination, motorChannel, velocity):
    setMotorTargetVelocityMsg = REVMsg.SetMotorTargetVelocity()
    setMotorTargetVelocityMsg.payload.motorChannel = motorChannel
    setMotorTargetVelocityMsg.payload.velocity = velocity
    commObj.sendAndReceive(setMotorTargetVelocityMsg, destination)


def getMotorTargetVelocity(commObj, destination, motorChannel):
    getMotorTargetVelocityMsg = REVMsg.GetMotorTargetVelocity()
    getMotorTargetVelocityMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorTargetVelocityMsg, destination)
    return packet.payload.velocity


def setMotorTargetPosition(commObj, destination, motorChannel, position, atTargetTolerance):
    setMotorTargetPositionMsg = REVMsg.SetMotorTargetPosition()
    setMotorTargetPositionMsg.payload.motorChannel = motorChannel
    setMotorTargetPositionMsg.payload.position = position
    setMotorTargetPositionMsg.payload.atTargetTolerance = atTargetTolerance
    commObj.sendAndReceive(setMotorTargetPositionMsg, destination)


def getMotorTargetPosition(commObj, destination, motorChannel):
    getMotorTargetPositionMsg = REVMsg.GetMotorTargetPosition()
    getMotorTargetPositionMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorTargetPositionMsg, destination)
    return (
     packet.payload.targetPosition, packet.payload.atTargetTolerance)


def getMotorAtTarget(commObj, destination, motorChannel):
    getMotorAtTargetMsg = REVMsg.GetMotorAtTarget()
    getMotorAtTargetMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorAtTargetMsg, destination)
    return packet.payload.atTarget


def getMotorEncoderPosition(commObj, destination, motorChannel):
    getMotorEncoderPositionMsg = REVMsg.GetMotorEncoderPosition()
    getMotorEncoderPositionMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getMotorEncoderPositionMsg, destination)
    val = int(packet.payload.currentPosition)
    bits = int(32)
    if val & 1 << bits - 1 != 0:
        val = val - (1 << bits)
    return val


def setMotorPIDCoefficients(commObj, destination, motorChannel, mode, p, i, d):
    setMotorPIDCoefficientsMsg = REVMsg.SetMotorPIDCoefficients()
    setMotorPIDCoefficientsMsg.payload.motorChannel = motorChannel
    setMotorPIDCoefficientsMsg.payload.mode = mode
    setMotorPIDCoefficientsMsg.payload.p = p * Q16
    setMotorPIDCoefficientsMsg.payload.i = i * Q16
    setMotorPIDCoefficientsMsg.payload.d = d * Q16
    commObj.sendAndReceive(setMotorPIDCoefficientsMsg, destination)


def getMotorPIDCoefficients(commObj, destination, motorChannel, mode):
    getMotorPIDCoefficientsMsg = REVMsg.GetMotorPIDCoefficients()
    getMotorPIDCoefficientsMsg.payload.motorChannel = motorChannel
    getMotorPIDCoefficientsMsg.payload.mode = mode
    packet = commObj.sendAndReceive(getMotorPIDCoefficientsMsg, destination)
    p = int(packet.payload.p) / Q16
    i = int(packet.payload.i) / Q16
    d = int(packet.payload.d) / Q16
    return (
     p, i, d)


def getBulkPIDData(commObj, destination, motorChannel):
    getBulkPIDDataMsg = REVMsg.GetBulkPIDData()
    getBulkPIDDataMsg.payload.motorChannel = motorChannel
    packet = commObj.sendAndReceive(getBulkPIDDataMsg, destination)
    return packet


def setCurrentPIDCoefficients(commObj, destination, motorChannel, p, i, d):
    getMotorPIDCoefficients(commObj, destination, motorChannel, 3, p, i, d)


def setVelocityPIDCoefficients(commObj, destination, motorChannel, p, i, d):
    setMotorPIDCoefficients(commObj, destination, motorChannel, 1, p, i, d)


def setPositionPIDCoefficients(commObj, destination, motorChannel, p, i, d):
    setMotorPIDCoefficients(commObj, destination, motorChannel, 2, p, i, d)


def getCurrentPIDCoefficients(commObj, destination, motorChannel):
    return getMotorPIDCoefficients(commObj, destination, motorChannel, 3)


def getVelocityPIDCoefficients(commObj, destination, motorChannel):
    return getMotorPIDCoefficients(commObj, destination, motorChannel, 1)


def getPositionPIDCoefficients(commObj, destination, motorChannel):
    return getMotorPIDCoefficients(commObj, destination, motorChannel, 2)
