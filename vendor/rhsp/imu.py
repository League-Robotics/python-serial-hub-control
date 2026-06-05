from rhsp.i2c import I2CDevice
from rhsp.internal.imu import IMUConstants
import time


class IMU(I2CDevice):

    def __init__(self, commObj, channel, destinationModule):
        I2CDevice.__init__(self, commObj, channel, destinationModule, IMUConstants.IMU_ADDRESS)

    def getDeviceID(self):
        return self.getRegisterValue(IMUConstants.CHIP_ID)

    def initSensor(self):
        for _ in range(3):
            self.setRegisterValue(IMUConstants.OPR_MODE, IMUConstants.CONFIGMODE)
            self.setRegisterValue(IMUConstants.PWR_MODE, IMUConstants.NORMAL)
            self.setRegisterValue(IMUConstants.SYS_TRIGGER, 128)
            self.setRegisterValue(IMUConstants.PAGE_ID, 0)
            self.setRegisterValue(IMUConstants.UNIT_SEL, IMUConstants.ACC_UNIT_MSS)
            self.setRegisterValue(IMUConstants.OPR_MODE, IMUConstants.IMUMODE)
            try:
                stat = self.getRegisterValue(IMUConstants.SYS_STAT)
            except AttributeError:
                stat = -1

            if stat == 5:
                break
            time.sleep(0.1)

    def getTemperature(self):
        return self.getRegisterValue(IMUConstants.TEMP)

    def getGyroData_X(self):
        return self.getTwoByteRegisterValue(IMUConstants.GYR_DATA_X_LSB)

    def getGyroData_Y(self):
        return self.getTwoByteRegisterValue(IMUConstants.GYR_DATA_Y_LSB)

    def getGyroData_Z(self):
        return self.getTwoByteRegisterValue(IMUConstants.GYR_DATA_Z_LSB)

    def getAccData_X(self):
        return self.getTwoByteRegisterValue(IMUConstants.ACC_DATA_X_LSB)

    def getAccData_Y(self):
        return self.getTwoByteRegisterValue(IMUConstants.ACC_DATA_Y_LSB)

    def getAccData_Z(self):
        return self.getTwoByteRegisterValue(IMUConstants.ACC_DATA_Z_LSB)

    def getMagData_X(self):
        return self.getTwoByteRegisterValue(IMUConstants.MAG_DATA_X_LSB)

    def getMagData_Y(self):
        return self.getTwoByteRegisterValue(IMUConstants.MAG_DATA_Y_LSB)

    def getMagData_Z(self):
        return self.getTwoByteRegisterValue(IMUConstants.MAG_DATA_Z_LSB)

    def getAllEuler(self):
        values = self.getSixByteRegisterValue(IMUConstants.EUL_H_LSB)
        return [360.0 * float(value) / 5760.0 for value in values]

    def getGravity(self):
        values = self.getSixByteRegisterValue(IMUConstants.GRV_DATA_X_LSB)
        return [float(value) / 100 for value in values]

    def getAllLinAccel(self):
        values = self.getSixByteRegisterValue(IMUConstants.LIA_DATA_X_LSB)
        return [float(value) / 1000 for value in values]

    def setRegisterValue(self, register, value):
        self.writeMultipleBytes(2, register + (value << 8))

    def getRegisterValue(self, register):
        self.writeByte(register)
        return self.readByte()

    def getTwoByteRegisterValue(self, register):
        self.writeByte(register)
        val = int(self.readMultipleBytes(2))
        bits = int(16)
        if val & 1 << bits - 1 != 0:
            val = val - (1 << bits)
        return val

    def getSixByteRegisterValue(self, register):
        self.writeByte(register)
        val = int(self.readMultipleBytes(6))
        bits = int(16)
        values = []
        for i in range(0, 3):
            it_val = val & 65535
            if it_val & 1 << bits - 1 != 0:
                it_val = it_val - (1 << bits)
            values.append(it_val)
            val = val >> 16

        return values