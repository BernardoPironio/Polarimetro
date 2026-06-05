from machine import Pin, I2C, ADC
from x9cxxx import X9Cxxx
import time

class Medicion:
    
    def __init__(self,sda = 0, scl = 1, fot_pin = 0, inc_pin = 14, ud_pin = 13,cs_pin = 15,freq = 200000,resistencia = 50):
        
        self.i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=freq)
        time.sleep_ms(200)
        self.fot = ADC(fot_pin)
        self.pot = X9Cxxx(inc_pin=inc_pin, ud_pin=ud_pin, cs_pin=cs_pin)
        self.pot.set(resistencia)
        
        self.ADDR       = 0x36
        self.STATUS_REG = 0x0B
        
    def medir(self,vueltas = 20, muestras_por_vuelta = 600):
        
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
    def medir_continuo(self, muestras_por_vuelta=600, max_fot=50000):

        voltaje    = [0] * max_fot
        t_voltaje  = [0] * max_fot
        vuelta_fot = [0] * max_fot

        idx_fot  = 0
        md_prev  = 0
        t_inicio = None
        t_ref    = time.ticks_us()
        vuelta   = 0

        # corre hasta llenar el buffer de fotodiodo
        while idx_fot < max_fot:
            try:
                s  = self.i2c.readfrom_mem(self.ADDR, self.STATUS_REG, 1)[0]
                md = (s >> 5) & 1

                if md == 1 and md_prev == 0:
                    ahora = time.ticks_us()
                    if t_inicio is not None:
                        dt = time.ticks_diff(ahora, t_inicio) // 1000
                        if dt > 5:
                            vuelta += 1
                    t_inicio = ahora

                md_prev = md

                if idx_fot < max_fot:
                    voltaje[idx_fot]    = self.fot.read_u16() * 3.3 / 65535
                    t_voltaje[idx_fot]  = time.ticks_diff(time.ticks_us(), t_ref)
                    vuelta_fot[idx_fot] = vuelta
                    idx_fot += 1

            except OSError:
                time.sleep_ms(100)

        return {
            "voltaje":    voltaje,
            "t_voltaje":  t_voltaje,
            "vuelta_fot": vuelta_fot,
            "n_vueltas":  vuelta,
            "n_fot":      idx_fot,
        }

    # ── Guardar resultado en CSV ─────────────────────────────────────────
    def guardar_csv(self, datos, archivo="datos.csv"):

        periodos  = datos.get("periodos",  [])
        t_vueltas = datos.get("t_vueltas", [])
        voltaje   = datos["voltaje"]
        t_voltaje = datos["t_voltaje"]
        vuelta_fot = datos["vuelta_fot"]
        n         = max(len(periodos), datos["n_fot"])

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
        
    # ── Calibrar resistencia ─────────────────────────────────────────    
    def calibrar_resistencia(self, vueltas=3, muestras_por_vuelta=600,
                             umbral_plano=0.05, max_iter=99):

        for iteracion in range(max_iter):
            datos   = self.medir(vueltas=vueltas, muestras_por_vuelta=muestras_por_vuelta)
            voltaje = datos["voltaje"]

            planos = 0
            saturacion = False
            for i in range(len(voltaje) - 1):
                if abs(voltaje[i+1] - voltaje[i]) < umbral_plano:
                    planos += 1
                    if planos >= 3:
                        saturacion = True
                        break
                else:
                    planos = 0

            if not saturacion:
                print("Resistencia OK:", self.pot.get())
                return self.pot.get()

            actual = self.pot.get()
            if actual < 99:
                self.pot.set(actual + 1)
            else:
                print("Resistencia maxima alcanzada")
                return actual
