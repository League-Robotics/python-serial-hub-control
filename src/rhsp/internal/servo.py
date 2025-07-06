from . import messages as REVMsg


def setServoConfiguration(commObj, destination, servoChannel, framePeriod):
    setServoConfigurationMsg = REVMsg.SetServoConfiguration()
    setServoConfigurationMsg.payload.servoChannel = servoChannel
    setServoConfigurationMsg.payload.framePeriod = framePeriod
    return commObj.sendAndReceive(setServoConfigurationMsg, destination)


def getServoConfiguration(commObj, destination, servoChannel):
    getServoConfigurationMsg = REVMsg.GetServoConfiguration()
    getServoConfigurationMsg.payload.servoChannel = servoChannel
    packet = commObj.sendAndReceive(getServoConfigurationMsg, destination)
    return packet.payload.framePeriod


def setServoPulseWidth(commObj, destination, servoChannel, pulseWidth):
    setServoPulseWidthMsg = REVMsg.SetServoPulseWidth()
    setServoPulseWidthMsg.payload.servoChannel = servoChannel
    setServoPulseWidthMsg.payload.pulseWidth = pulseWidth
    return commObj.sendAndReceive(setServoPulseWidthMsg, destination)


def getServoPulseWidth(commObj, destination, servoChannel):
    getServoPulseWidthMsg = REVMsg.GetServoPulseWidth()
    getServoPulseWidthMsg.payload.servoChannel = servoChannel
    packet = commObj.sendAndReceive(getServoPulseWidthMsg, destination)
    return packet.payload.pulseWidth


def setServoEnable(commObj, destination, servoChannel, enable):
    setServoEnableMsg = REVMsg.SetServoEnable()
    setServoEnableMsg.payload.servoChannel = servoChannel
    setServoEnableMsg.payload.enable = enable
    return commObj.sendAndReceive(setServoEnableMsg, destination)


def getServoEnable(commObj, destination, servoChannel):
    getServoEnableMsg = REVMsg.GetServoEnable()
    getServoEnableMsg.payload.servoChannel = servoChannel
    packet = commObj.sendAndReceive(getServoEnableMsg, destination)
    return packet.payload.enabled


