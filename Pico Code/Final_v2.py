from machine import Pin, I2C, ADC
from x9cxxx import X9Cxxx
import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=200000)
time.sleep_ms(200)

fot = ADC(0)
pot = X9Cxxx(inc_pin=14, ud_pin=13, cs_pin=15)
pot.set(0)
print(pot.get())

ADDR        = 0x36
STATUS_REG  = 0x0B
MAX_VUELTAS = 20
MAX_FOT     = MAX_VUELTAS * 600

periodos  = [0] * MAX_VUELTAS
t_vueltas = [0] * MAX_VUELTAS
voltaje   = [0] * MAX_FOT
t_voltaje = [0] * MAX_FOT
vuelta_fot = [0] * MAX_FOT  # a qué vuelta pertenece cada lectura

idx        = 0
idx_fot    = 0
md_prev    = 0
t_inicio   = None
t_ref      = time.ticks_us()  # tiempo cero global

while idx < MAX_VUELTAS:
    try:
        s  = i2c.readfrom_mem(ADDR, STATUS_REG, 1)[0]
        md = (s >> 5) & 1

        if md == 1 and md_prev == 0:
            ahora = time.ticks_us()
            if t_inicio is not None:
                dt = time.ticks_diff(ahora, t_inicio) // 1000  # a ms
                if dt > 5:
                    periodos[idx]  = dt
                    t_vueltas[idx] = time.ticks_diff(ahora, t_ref)
                    idx += 1
            t_inicio = ahora

        md_prev = md

        if idx_fot < MAX_FOT:
            voltaje[idx_fot]    = fot.read_u16() * 3.3 / 65535
            t_voltaje[idx_fot]  = time.ticks_diff(time.ticks_us(), t_ref)
            vuelta_fot[idx_fot] = idx  # vuelta actual al momento de la lectura
            idx_fot += 1

    except OSError:
        time.sleep_ms(100)

with open("datos.csv", "w") as f:
    f.write("periodo_ms,t_vuelta_us,voltaje,t_voltaje_us,vuelta\n")
    n = max(idx, idx_fot)
    for i in range(n):
        periodo  = periodos[i]   if i < idx     else ""
        t_vuelta = t_vueltas[i]  if i < idx     else ""
        volt     = voltaje[i]    if i < idx_fot else ""
        t_volt   = t_voltaje[i]  if i < idx_fot else ""
        vuelta   = vuelta_fot[i] if i < idx_fot else ""
        f.write("{},{},{},{},{}\n".format(periodo, t_vuelta, volt, t_volt, vuelta))

print("archivo creado")