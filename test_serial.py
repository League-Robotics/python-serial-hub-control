from rshp import client
from serial.tools import list_ports
import time
from rshp.rshp_serial import comPort

ports = comPort.enumerate()

print("Available COM Ports:")
for port in ports:
    print(f"Port: {port.name}, Serial Number: {port.sn}")

    