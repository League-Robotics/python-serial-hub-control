
from rhsp import Client, Module
from rhsp.internal.messages import LEDPattern
import time
from rhsp.i2c import I2CDevice
from rhsp.distance import Distance2m

import sys

c = Client()

c.open()

for m in c.discovery():
    print("Status: ", m.getStatus())

m.i2cChannels[0].add

sys.exit(0)

sensor = Distance2m(c, 0, m.getAddress(), False)
isSensor = sensor.Is2mDistanceSensor()
print(str(isSensor))
if isSensor:
    sensor.initialize()
    for i in range(0, 500):
        print(sensor.readRangeContinuousMillimeters())

else:
    print('No sensor found, quitting...')