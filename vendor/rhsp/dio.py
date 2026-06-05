
from rhsp.internal.dio import setSingleDIOOutput, getSingleDIOInput, setDIODirection, getDIODirection

class DIOPin:

    def __init__(self, commObj, pinNumber, destinationModule):
        self.destinationModule = destinationModule
        self.pinNumber = pinNumber
        self.commObj = commObj

    def setDestination(self, destinationModule):
        self.destinationModule = destinationModule

    def getDestination(self):
        return self.destinationModule

    def setPinNumber(self, pinNumber):
        self.pinNumber = pinNumber

    def getPinNumber(self):
        return self.pinNumber

    def setOutput(self, value):
        setSingleDIOOutput(self.commObj, self.destinationModule, self.pinNumber, value)

    def getInput(self):
        return getSingleDIOInput(self.commObj, self.destinationModule, self.pinNumber)

    def setAsOutput(self):
        setDIODirection(self.commObj, self.destinationModule, self.pinNumber, 1)

    def setAsInput(self):
        setDIODirection(self.commObj, self.destinationModule, self.pinNumber, 0)

    def getDirection(self):
        getDIODirection(self.commObj, self.destinationModule, self.pinNumber)