from machine import Pin, I2C, SoftI2C
from time import sleep

i2c = I2C(0,sda = Pin(0),scl = Pin(1),freq = 10000)
print('Devices found : ',i2c.scan())

sda = Pin(2,Pin.IN,Pin.PULL_UP)
scl = Pin(3,Pin.IN,Pin.PULL_UP)

ADDR = 0x36
STATUS = 0x0B

# Angle regystry
ANGLE_H = 0x0E
ANGLE_L = 0x0F

# Read if magnet is present
def check_magnet():
    while True:
        try:
            s = i2c.readfrom_mem(ADDR, STATUS, 1)[0]

            md = (s >> 5) & 1 # Magnet detected
            ml = (s >> 4) & 1 # Magnet too low
            mh = (s >> 3) & 1 # Magnet too high

            print("STATUS:", s, "MD:", md, "ML:", ml, "MH:", mh)

        except OSError as e:
            print("Error I2C:", e)

        sleep(0.2)

# Read angle

def read_angle():
    while True:
        try:
            data = i2c.readfrom_mem(ADDR, ANGLE_H, 2)
            raw = ((data[0] << 8) | data[1]) & 0x0FFF
            grados = raw * 360 / 4096

            print(round(grados, 2))

        except OSError as e:
            print("Error:", e)

        sleep(0.1)
        


