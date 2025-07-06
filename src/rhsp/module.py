from . import REVI2C, adc, motors
from .dio import DIOPin
from .servo import Servo
from .internal import dio
from .internal.messages import LEDPattern
from typing import TYPE_CHECKING, List, Any, Tuple

if TYPE_CHECKING:
    from .client import Client

class Module:

    def __init__(self, client: "Client", address: int, parent: Any):
        self.client: "Client" = client
        self.address: int = address
        self.parent: Any = parent
        self.motors: List[motors.Motor] = []
        self.servos: List[Servo] = []
        self.i2cChannels: List[REVI2C.I2CChannel] = []
        self.adcPins: List[adc.ADCPin] = []
        self.dioPins: List[DIOPin] = []

    def init_periphs(self) -> None:
        for i in range(0, 4):
            self.motors.append(motors.Motor(self.client, i, self.address))
            self.motors[-1].setMode(0, 1)
            self.motors[-1].setPower(0)
            self.i2cChannels.append(REVI2C.I2CChannel(self.client, i, self.address))

        for j in range(0, 8):
            self.dioPins.append(DIOPin(self.client, j, self.address))

        for k in range(0, 6):
            self.servos.append(Servo(self.client, k, self.address))
            self.servos[-1].init()

        for l in range(0, 4):
            self.adcPins.append(adc.ADCPin(self.client, l, self.address))

    def killSwitch(self) -> None:
        for i in range(0, 4):
            self.motors[i].disable()

        for j in range(0, 8):
            pass

        for k in range(0, 6):
            self.servos[k].disable()

        for l in range(0, 15):
            pass

    def getParentStatus(self) -> Any:
        return self.parent

    def getAddress(self) -> int:
        return self.address

    def getStatus(self) -> Any:
        return self.client.getModuleStatus(self.address)

    def getModuleAddress(self) -> int:
        return self.address

    def keep_alive(self) -> Any:
        return self.client.keepAlive(self.address)

    def sendFailSafe(self) -> None:
        self.client.failSafe(self.address)

    def setAddress(self, newAddress: int) -> None:
        self.client.setNewModuleAddress(self.address, newAddress)
        self.address = newAddress
        for motor in self.motors:
            motor.setDestination(newAddress)

        for servo in self.servos:
            servo.setDestination(newAddress)

        for i2cChannel in self.i2cChannels:
            i2cChannel.setDestination(newAddress)

        for p in self.adcPins:
            p.setDestination(newAddress)

        for p in self.dioPins:
            p.setDestination(newAddress)

    def getInterface(self, interface: Any) -> Any:
        return self.client.queryInterface(self.address, interface)

    def setLEDColor(self, red: int, green: int, blue: int) -> None:
        self.client.setModuleLEDColor(self.address, red, green, blue)

    def getLEDColor(self) -> Tuple[int, int, int]:
        return self.client.getModuleLEDColor(self.address)

    def setLEDPattern(self, pattern: LEDPattern) -> Any:
        """ Example:
      from REVmessages import LEDPattern

      hub = REVModules()
      my_pattern = LEDPattern()
      my_pattern.set_step(0, 255, 0, 0, 10) # set first step to red for 1 second
      my_pattern.set_step(1, 0, 255, 0, 10) # set second step to green for 1 second
      hub.REVModules[0].setLEDPattern(my_pattern)
      hub.REVModules[0].keepAlive()
      """
        return self.client.setModuleLEDPattern(self.address, pattern)

    def setLogLevel(self, group, verbosity):
        self.client.debugLogLevel(self.address, group, verbosity)

    def getBulkData(self):
        return self.client.getBulkInputData(self.address)

    def enableCharging(self):
        self.client.phoneChargeControl(self.address, 1)

    def disableCharging(self):
        self.client.phoneChargeControl(self.address, 0)

    def chargingEnabled(self):
        return self.client.phoneChargeQuery(self.address)

    def debugOutput(self, length, hint):
        self.client.injectDataLogHint(self.address, length, hint)

    def setAllDIO(self, values):
        dio.setAllDIOOutputs(self.address, values)

    def getAllDIO(self):
        return dio.getAllDIOInputs(self.client, self.address)

    def getVersionString(self):
        versionRaw = '' + self.client.readVersionString(self.address)
        versionStr = ''
        for i in range(0, int(len(versionRaw) / 2)):
            tmpHex = int(str(versionRaw)[i * 2] + str(versionRaw)[i * 2 + 1], 16)
            versionStr = versionStr + chr(tmpHex)
        return versionStr

    def setIMUBlockReadConfig(self, startRegister, numberOfBytes, readInterval_ms):
        REVI2C.imuBlockReadConfig(self.address, startRegister, numberOfBytes, readInterval_ms)

    def getIMUBlockReadConfig(self):
        return REVI2C.imuBlockReadQuery(self.address)

    def getBulkMotorData(self):
        return self.client.getBulkMotorData(self.address)

    def getBulkADCData(self):
        return self.client.getBulkADCData(self.address)

    def getBulkI2CData(self):
        return self.client.getBulkI2CData(self.address)

    def getBulkServoData(self):
        return self.client.getBulkServoData(self.address)