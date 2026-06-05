from  . import messages as REVMsg


def setSingleDIOOutput(commObj, destination, dioPin, value):
    setSingleDIOOutput = REVMsg.SetSingleDIOOutput()
    setSingleDIOOutput.payload.dioPin = dioPin
    setSingleDIOOutput.payload.value = value
    commObj.sendAndReceive(setSingleDIOOutput, destination)


def setAllDIOOutputs(commObj, destination, values):
    setAllDIOOutputs = REVMsg.SetAllDIOOutputs()
    setAllDIOOutputs.payload.values = values
    commObj.sendAndReceive(setAllDIOOutputs, destination)


def setDIODirection(commObj, destination, dioPin, directionOutput):
    setDIODirection = REVMsg.SetDIODirection()
    setDIODirection.payload.dioPin = dioPin
    setDIODirection.payload.directionOutput = directionOutput
    commObj.sendAndReceive(setDIODirection, destination)


def getDIODirection(commObj, destination, dioPin):
    getDIODirection = REVMsg.GetDIODirection()
    getDIODirection.payload.dioPin = dioPin
    packet = commObj.sendAndReceive(getDIODirection, destination)
    return packet.payload.directionOutput


def getSingleDIOInput(commObj, destination, dioPin):
    getSingleDIOInput = REVMsg.GetSingleDIOInput()
    getSingleDIOInput.payload.dioPin = dioPin
    packet = commObj.sendAndReceive(getSingleDIOInput, destination)
    return packet.payload.inputValue


def getAllDIOInputs(commObj, destination):
    getAllDIOInputs = REVMsg.GetAllDIOInputs()
    packet = commObj.sendAndReceive(getAllDIOInputs, destination)
    return packet.payload.inputValues
