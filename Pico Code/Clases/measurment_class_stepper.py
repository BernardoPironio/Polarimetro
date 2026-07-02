from machine import Pin, I2C, ADC
from x9cxxx import X9Cxxx
import time

class Medicion:
    
    def __init__(self, sda=0, scl=1, fot_pin=0, freq=200000):
        
        self.i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=freq)
        time.sleep_ms(200)
        self.fot = ADC(fot_pin)
        
        self.ADDR       = 0x36
        self.STATUS_REG = 0x0B

    # ── Medición con número fijo de vueltas ──────────────────────────────
    def medir(self, vueltas=20, muestras_por_vuelta=600):
        
        MAX_FOT = vueltas * muestras_por_vuelta
        
        periodos   = [0] * vueltas
        t_vueltas  = [0] * vueltas
        voltaje    = [0] * MAX_FOT
        t_voltaje  = [0] * MAX_FOT
        vuelta_fot = [0] * MAX_FOT
        
        idx      = 0
        idx_fot  = 0
        md_prev  = 0
        t_inicio = None
        t_ref    = time.ticks_us()
        
        while idx < vueltas:
            try:
                s  = self.i2c.readfrom_mem(self.ADDR, self.STATUS_REG, 1)[0]
                md = (s >> 5) & 1

                if md == 1 and md_prev == 0:
                    ahora = time.ticks_us()
                    if t_inicio is not None:
                        dt = time.ticks_diff(ahora, t_inicio) // 1000
                        if dt > 5:
                            periodos[idx]  = dt
                            t_vueltas[idx] = time.ticks_diff(ahora, t_ref)
                            idx += 1
                    t_inicio = ahora

                md_prev = md

                if idx_fot < MAX_FOT:
                    voltaje[idx_fot]    = self.fot.read_u16() * 3.3 / 65535
                    t_voltaje[idx_fot]  = time.ticks_diff(time.ticks_us(), t_ref)
                    vuelta_fot[idx_fot] = idx
                    idx_fot += 1

            except OSError:
                time.sleep_ms(100)

        return {
            "periodos":   periodos,
            "t_vueltas":  t_vueltas,
            "voltaje":    voltaje[:idx_fot],
            "t_voltaje":  t_voltaje[:idx_fot],
            "vuelta_fot": vuelta_fot[:idx_fot],
            "n_vueltas":  idx,
            "n_fot":      idx_fot,
        }

    # ── Medición continua sin límite de vueltas ──────────────────────────
    def medir_continuo(self):

        md_prev = 0
        t_inicio = None

        vuelta = 0

        tiempos = []
        voltajes = []

        while True:

            try:

                ahora = time.ticks_us()

                s = self.i2c.readfrom_mem(
                    self.ADDR,
                    self.STATUS_REG,
                    1
                )[0]

                md = (s >> 5) & 1

                # Inicio de una nueva vuelta
                if md == 1 and md_prev == 0:

                    if t_inicio is not None:

                        dt = time.ticks_diff(
                            ahora,
                            t_inicio
                        ) // 1000

                        if dt > 5:

                            # Enviar todas las muestras de la vuelta
                            if len(tiempos) > 0:

                                t0 = tiempos[0]

                                for tt, vv in zip(tiempos, voltajes):

                                    print("{},{},{}".format(
                                        tt - t0,
                                        vv,
                                        vuelta
                                    ))

                            vuelta += 1

                            tiempos = []
                            voltajes = []

                    t_inicio = ahora

                md_prev = md

                tiempos.append(ahora)

                voltajes.append(
                    self.fot.read_u16() * 3.3 / 65535
                )

            except OSError:

                time.sleep_ms(10)

    # ── Guardar resultado en CSV ─────────────────────────────────────────
    def guardar_csv(self, datos, archivo="datos.csv"):

        periodos   = datos.get("periodos",  [])
        t_vueltas  = datos.get("t_vueltas", [])
        voltaje    = datos["voltaje"]
        t_voltaje  = datos["t_voltaje"]
        vuelta_fot = datos["vuelta_fot"]
        n          = max(len(periodos), datos["n_fot"])

        with open(archivo, "w") as f:
            f.write("periodo_ms,t_vuelta_us,voltaje,t_voltaje_us,vuelta\n")
            for i in range(n):
                periodo  = periodos[i]   if i < len(periodos)  else ""
                t_vuelta = t_vueltas[i]  if i < len(t_vueltas) else ""
                volt     = voltaje[i]    if i < datos["n_fot"] else ""
                t_volt   = t_voltaje[i]  if i < datos["n_fot"] else ""
                vuelta   = vuelta_fot[i] if i < datos["n_fot"] else ""
                f.write("{},{},{},{},{}\n".format(
                    periodo, t_vuelta, volt, t_volt, vuelta))

        print("archivo creado:", archivo)
