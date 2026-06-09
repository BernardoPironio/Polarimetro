from machine import Pin, I2C, ADC
from x9cxxx import X9Cxxx
import time

class Medicion:
    
    def __init__(self, sda=0, scl=1, fot_pin=0, inc_pin=14, ud_pin=13, cs_pin=15, freq=200000):
        
        self.i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=freq)
        time.sleep_ms(200)
        self.fot = ADC(fot_pin)
        self.pot = X9Cxxx(inc_pin=inc_pin, ud_pin=ud_pin, cs_pin=cs_pin)
        
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

        md_prev  = 0
        t_inicio = None
        t_ref    = time.ticks_us()
        vuelta   = 0

        while True:

            try:

                s = self.i2c.readfrom_mem(
                    self.ADDR,
                    self.STATUS_REG,
                    1
                )[0]

                md = (s >> 5) & 1

                if md == 1 and md_prev == 0:

                    ahora = time.ticks_us()

                    if t_inicio is not None:

                        dt = time.ticks_diff(
                            ahora,
                            t_inicio
                        ) // 1000

                        if dt > 5:
                            vuelta += 1

                    t_inicio = ahora

                md_prev = md

                volt = self.fot.read_u16() * 3.3 / 65535

                t = time.ticks_diff(
                    time.ticks_us(),
                    t_ref
                )

                print("{},{},{}".format(
                    t,
                    volt,
                    vuelta
                ))

            except OSError:
                time.sleep_ms(100)

    # ── Calibración automática del potenciómetro ─────────────────────────
    def calibrar(self, delta=0.05, vueltas=3, muestras_por_vuelta=600, max_pos=99):
        """
        Busca la posición del potenciómetro donde la señal del fotodiodo
        no satura. Satura = todas las diferencias entre muestras < delta.
        Aumenta el pot de a 1 hasta encontrar un valor válido o llegar a max_pos.
        """
        pos = self.pot.get() or 0
        self.pot.set(pos)

        while pos <= max_pos:
            self.pot.set(pos)
            datos = self.medir(vueltas=vueltas, muestras_por_vuelta=muestras_por_vuelta)

            voltaje = datos["voltaje"]

            diferencias = [
                abs(voltaje[i+1] - voltaje[i])
                for i in range(len(voltaje) - 1)
            ]
            max_diff = max(diferencias) if diferencias else 0
            satura   = max_diff < delta

            print("pos={:3d} | max_diff={:.4f}V | {}".format(
                pos, max_diff, "SATURA" if satura else "OK"))

            if not satura:
                return {
                    "posicion_final": pos,
                    "satura":         False,
                    "datos":          datos,
                }

            pos += 1

        # Se agotaron las posiciones y sigue saturando
        return {
            "posicion_final": pos - 1,
            "satura":         True,
            "datos":          datos,
        }

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