
from rshp import Client, Module
from rshp.messages import LEDPattern
import time

c = Client()

c.open()


for m in c.discovery():
    print("Status: ", m.getStatus())

lp  = LEDPattern(
    [
    (0, 0, 0, 1),  # Off
    (255, 0, 0, 1),  # Red
    (0, 255, 0, 1),  # Green
    (0, 0, 255, 1),  # Blue
    (255, 255, 0, 1),  # Yellow
    (255, 0, 255, 1),  # Magenta
    (0, 255, 255, 1),  # Cyan
    (255, 255, 255, 1)   # White
    ]
)

#m.setLEDPattern(lp)

m.init_periphs()
m.motors[0].init()
m.motors[1].init()


max_power = 2**15 - 1  # Maximum power for the motor

for i in range(10):
    
    m.keep_alive()
   
    d = m.getAllDIO()

    m.motors[0].setPower( (i/10)*max_power )  # Alternate power
    m.motors[1].setPower( (i/10)*max_power )  # Alternate power

    print( i, d[1], d[3])
    time.sleep(1)