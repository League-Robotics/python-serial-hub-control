
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

color = m.i2cChannels[1].addColorSensor()

color.initSensor()
for i in range(0, 5000):
    d = color.getHueName()
    print(f"Color: {d}")
    time.sleep(.25)