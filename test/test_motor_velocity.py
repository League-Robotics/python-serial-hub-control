
from rhsp import Client, Module
from rhsp.internal.messages import LEDPattern
from rhsp.motors import MODE_CONSTANT_VELOCITY
import time

c = Client()

c.open()

for m in c.discovery():
    print("Status: ", m.getStatus())

m.init_periphs()
m.motors[0].init()
m.motors[0].setMode(MODE_CONSTANT_VELOCITY, 1)


m.motors[1].init()

max_power = 2**15 - 1  # Maximum power for the motor

m.keep_alive()

def speed_range():
    for i in range(0,3000, 100):
        
        m.motors[0].setTargetVelocity( i )  
     
        print(f"m0_tv: {i:6d} m0_v {m.motors[0].getVelocity():6.2f}")


        time.sleep(1)

speed_range()