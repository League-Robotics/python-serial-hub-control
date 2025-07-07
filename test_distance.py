
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

dist = m.i2cChannels[0].addDistanceSensor()
dist.initialize()
for i in range(0, 500):
    dist = dist.readRangeContinuousMillimeters()

