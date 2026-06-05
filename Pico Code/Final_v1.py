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
MAX_FOT     = MAX_VUELTAS * 600  # ~200 lecturas por vuelta máximo

periodos        = [0] * MAX_VUELTAS
t_vueltas       = [0] * MAX_VUELTAS  # timestamp de cada vuelta
voltaje         = [0] * MAX_FOT
t_voltaje       = [0] * MAX_FOT      # timestamp de cada lectura fotodiodo

idx         = 0
idx_fot     = 0
md_prev     = 0
t_inicio    = None

while idx < MAX_VUELTAS:
    try:
        s  = i2c.readfrom_mem(ADDR, STATUS_REG, 1)[0]
        md = (s >> 5) & 1

        if md == 1 and md_prev == 0:
            ahora = time.ticks_us()
            if t_inicio is not None:
                dt = time.ticks_diff(ahora, t_inicio)
                if dt > 5:
                    periodos[idx]  = dt
                    t_vueltas[idx] = ahora
                    idx += 1
            t_inicio = ahora

        md_prev = md

        # leer fotodiodo en cada iteracion del loop
        if idx_fot < MAX_FOT:
            voltaje[idx_fot]   = fot.read_u16() * 3.3 / 65535
            t_voltaje[idx_fot] = time.ticks_us()
            idx_fot += 1

    except OSError:
        time.sleep_ms(100)

with open("datos.csv", "w") as f:
    # Cabecera
    f.write("periodo,t_vuelta,voltaje,t_voltaje\n")

    # Número máximo de filas
    n = max(idx, idx_fot)

    for i in range(n):
        periodo = periodos[i] if i < idx else ""
        t_vuelta = t_vueltas[i] if i < idx else ""
        volt = voltaje[i] if i < idx_fot else ""
        t_volt = t_voltaje[i] if i < idx_fot else ""

        f.write("{},{},{},{}\n".format(
            periodo, t_vuelta, volt, t_volt
        ))
        
print("archivo creado")