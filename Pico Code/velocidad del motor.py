from machine import Pin, I2C, ADC
from x9cxxx import X9Cxxx
import time

# I2C - Encoder a modo de tick
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=200000)
time.sleep_ms(200)

# ADC - Fotodiodo
fot = ADC(0)
pot = X9Cxxx(inc_pin=14, ud_pin=13, cs_pin=15)

pot.set(0)
position = pot.get()
print(position)


ADDR        = 0x36
STATUS_REG  = 0x0B
MAX_VUELTAS = 50

periodos    = [0] * MAX_VUELTAS
voltaje     = [0] *MAX_VUELTAS*100

idx         = 0
idx_fot     = 0
md_prev     = 0
t_inicio    = None

while idx < MAX_VUELTAS:
    try:
        s  = i2c.readfrom_mem(ADDR, STATUS_REG, 1)[0]
        md = (s >> 5) & 1

        if md == 1 and md_prev == 0:
            ahora = time.ticks_ms()
            if t_inicio is not None:
                dt = time.ticks_diff(ahora, t_inicio)
                if dt > 5:
                    periodos[idx] = dt
                    idx += 1
            t_inicio = ahora

        md_prev = md
    except OSError:
        time.sleep_ms(100)

print(periodos[:idx])
print('Velocidad :', 1000/periodos[3], ' vueltas por segundo')