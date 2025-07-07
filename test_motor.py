
from rhsp import Client, Module
from rhsp.internal.messages import LEDPattern
import time

c = Client()

c.open()


for m in c.discovery():
    print("Status: ", m.getStatus())

m.init_periphs()
m.motors[0].init()
m.motors[1].init()


max_power = 2**15 - 1  # Maximum power for the motor

for i in range(10):
    

    m.motors[0].setPower( (i/10)*max_power )  # Alternate power
    m.motors[1].setPower( (i/10)*max_power )  # Alternate power

    time.sleep(1)