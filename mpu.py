# mpu_test.py
import smbus
import time

bus = smbus.SMBus(1)
# prueba ambas direcciones si no sabes cuál
for addr in (0x68, 0x69):
    try:
        who = bus.read_byte_data(addr, 0x75)
        print(f"Dirección 0x{addr:02x} -> WHO_AM_I = 0x{who:02x}")
    except Exception as e:
        print(f"Dirección 0x{addr:02x} -> error: {e}")