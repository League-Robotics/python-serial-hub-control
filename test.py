
from rshp import Client, Module
import time

c = Client()

c.open()


for m in c.discovery():
    print("Status: ", m.getStatus())


time.sleep(10)