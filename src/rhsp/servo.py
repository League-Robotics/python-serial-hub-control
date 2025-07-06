from rhsp.internal.servo import (getServoConfiguration, getServoEnable, getServoPulseWidth, 
                                 setServoConfiguration, setServoEnable, setServoPulseWidth)


class Servo:

    def __init__(self, commObj, channel, destinationModule):
        self.commObj = commObj
        self.destinationModule = destinationModule
        self.channel = channel

    def setDestination(self, destinationModule):
        self.destinationModule = destinationModule

    def getDestination(self):
        return self.destinationModule

    def setChannel(self, channel):
        self.channel = channel

    def getChannel(self):
        return self.channel

    def setPeriod(self, period):
        setServoConfiguration(self.commObj, self.destinationModule, self.channel, period)

    def getPeriod(self):
        return getServoConfiguration(self.commObj, self.destinationModule, self.channel)

    def setPulseWidth(self, pulseWidth):
        setServoPulseWidth(self.commObj, self.destinationModule, self.channel, pulseWidth)

    def getPulseWidth(self):
        return getServoPulseWidth(self.commObj, self.destinationModule, self.channel)

    def enable(self):
        errorCode = setServoEnable(self.commObj, self.destinationModule, self.channel, 1)

    def disable(self):
        setServoEnable(self.commObj, self.destinationModule, self.channel, 0)

    def isEnabled(self):
        return getServoEnable(self.commObj, self.destinationModule, self.channel)

    def init(self):
        self.setPeriod(20000)