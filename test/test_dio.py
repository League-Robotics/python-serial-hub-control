
from rhsp import Client, Module
from rhsp.internal.messages import LEDPattern
import time

c = Client()

c.open()


for m in c.discovery():
    print("Status: ", m.getStatus())

m.init_periphs()


m.keep_alive()

def speed_range():
    for i in range(100):
        
        d = m.getAllDIO()

        for i in range(4):
            print(f"{d[i]}", end=" ")
        print()

        time.sleep(1)

speed_range()