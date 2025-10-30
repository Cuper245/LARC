from machine import I2C, Pin
import time

# Configura el bus I2C (ajusta los pines según tu placa)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

MPU_ADDR = 0x68  # Dirección si AD0 = GND

# Inicializa el MPU6050
i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')  # Despierta el sensor

def read_accel_x():
    data = i2c.readfrom_mem(MPU_ADDR, 0x3B, 2)  # Registros ACCEL_XOUT_H y _L
    value = int.from_bytes(data, 'big', signed=True)
    return value

while True:
    x = read_accel_x()
    print("Aceleración X:", x)
    time.sleep(0.5)