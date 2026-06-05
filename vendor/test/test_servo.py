
from rhsp import Client, Module
from rhsp.internal.messages import LEDPattern
import time

c = Client()

c.open()

for m in c.discovery():
    print("Status: ", m.getStatus())

servo = m.servos[0]
servo.setPeriod(20000)
servo.setPulseWidth(1500)
servo.enable()

angles = list(range(180))

angles += reversed(angles)

for a in angles:
    d = m.getAllDIO()
    print(f"Angle  {a:3d} DIO: {d[1]} {d[3]}")
    servo.setAngle(a)

    #time.sleep(.0001)