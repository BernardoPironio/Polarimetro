from machine import Pin, PWM, I2C, Timer, ADC
from math import sin, pi
from array import array
import time
import micropython
import _thread
import time
import gc

micropython.alloc_emergency_exception_buf(100)

# =========================================================
# 1. PERIFÉRICOS
# =========================================================
fotodiodo = ADC(Pin(26))
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=10000)
ADDR   = 0x36
STATUS = 0x0B
i2c_buf = bytearray(1)

PIN_U, PIN_V, PIN_W, PIN_EN = 2, 3, 4, 5
pwm_u = PWM(Pin(PIN_U))
pwm_v = PWM(Pin(PIN_V))
pwm_w = PWM(Pin(PIN_W))

PWM_FREQ = 100000
pwm_u.freq(PWM_FREQ)
pwm_v.freq(PWM_FREQ)
pwm_w.freq(PWM_FREQ)

en = Pin(PIN_EN, Pin.OUT)
en.value(0)

# =========================================================
# 2. PARÁMETROS Y TABLAS
# =========================================================
V_SUPPLY    = 24.0
V_AMPLITUDE = 6.0
TABLE_SIZE  = 360
UPDATE_HZ   = 5500
STEP        = 4

HALF_SUPPLY = V_SUPPLY / 2.0
SCALE       = 65535.0 / V_SUPPLY

lut_u = array('H', [0] * TABLE_SIZE)
lut_v = array('H', [0] * TABLE_SIZE)
lut_w = array('H', [0] * TABLE_SIZE)

for i in range(TABLE_SIZE):
    th = 2.0 * pi * i / TABLE_SIZE
    ua = V_AMPLITUDE * sin(th)                + HALF_SUPPLY
    ub = V_AMPLITUDE * sin(th - 2.0*pi/3.0)   + HALF_SUPPLY
    uc = V_AMPLITUDE * sin(th - 4.0*pi/3.0)   + HALF_SUPPLY
    lut_u[i] = int(max(0, min(V_SUPPLY, ua)) * SCALE)
    lut_v[i] = int(max(0, min(V_SUPPLY, ub)) * SCALE)
    lut_w[i] = int(max(0, min(V_SUPPLY, uc)) * SCALE)

# Compartido entre núcleos — array es seguro para acceso cruzado
state    = array('I', [0])   # state[0] = phase_idx
running  = array('I', [1])   # running[0] = 1 mientras corre
periodos = array('I', [0] * 1000)

# =========================================================
# 3. NÚCLEO 1 — motor loop con hard timer propio
# =========================================================
def motor_tick(timer, _u=pwm_u, _v=pwm_v, _w=pwm_w,
               _lu=lut_u, _lv=lut_v, _lw=lut_w,
               _st=state, _step=STEP, _sz=TABLE_SIZE):
    idx = _st[0]
    _u.duty_u16(_lu[idx])
    _v.duty_u16(_lv[idx])
    _w.duty_u16(_lw[idx])
    _st[0] = (idx + _step) % _sz

def core1_motor():
    tim = Timer()
    # Hard timer en núcleo 1 — tiene su propio contexto Python, funciona bien
    tim.init(freq=UPDATE_HZ, mode=Timer.PERIODIC, callback=motor_tick, hard=True)
    while running[0]:
        time.sleep_ms(100)   # solo mantiene el núcleo vivo
    tim.deinit()
    pwm_u.duty_u16(0)
    pwm_v.duty_u16(0)
    pwm_w.duty_u16(0)

# =========================================================
# 4. MAIN (núcleo 0)
# =========================================================
count   = 0
md_prev = 0
idx_adc  = 0

t0       = time.ticks_us()
t_antes_us = time.ticks_us()

N        = 5000
data_adc = array('H', [0] * N)
times    = array('I', [0] * N)
iman_pulse = array('H', [0] * N)
periodo_us = array('I', [0] * N)


last_debug = time.ticks_ms()

interrupt = False

try:
    gc.collect()
    en.value(1)
    state[0]   = 0
    running[0] = 1

    print("Iniciando motor en núcleo 1...")
    _thread.start_new_thread(core1_motor, ())
    time.sleep_ms(100)  # dar tiempo a que arranque el timer
    print("Motor corriendo. Núcleo 0 libre para sensores.")
    
    idx_adc = 0
    t0 = time.ticks_us()
    t_antes_us = 0

    while idx_adc < N:
        now_us = time.ticks_diff(time.ticks_us(),t0)
        
        times[idx_adc] = now_us
        
        i2c.readfrom_mem_into(ADDR, STATUS, i2c_buf)
        md = (i2c_buf[0] >> 5) & 1
        

        if md == 1 and md_prev == 0:
            iman_pulse[idx_adc] = 1
            
            if count == 0:
                periodo = 0
            else:
                periodo = time.ticks_diff(now_us,t_antes_us)
            
            periodo_us[idx_adc] = periodo
            t_antes_us = now_us
            count  += 1

        md_prev = md

        data_adc[idx_adc] = fotodiodo.read_u16()
        idx_adc += 1

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_debug) > 1000:
            last_debug = now_ms
            print("Pulsos:", count,
                  "Mem:", gc.mem_free(),
                  "Fase:", state[0],
                  "Intensidad:", data_adc[idx_adc - 1])
            
            
except KeyboardInterrupt:
    interrupt = True
    print("\nDetenido por usuario")

finally:
    # Apagar motor y núcleo 1
    running[0] = 0
    try:
        time.sleep_ms(200)
    except KeyboardInterrupt:
        pass

    en.value(0)
    pwm_u.duty_u16(0)
    pwm_v.duty_u16(0)
    pwm_w.duty_u16(0)

    print("Sistema apagado. Pulsos totales:", count)

    if interrupt:
        print("Guardando datos...")

        try:
            with open("datos_motor.csv", "w") as f:
                f.write("time_us,adc_raw,iman,periodo_us\n")

                n_adc = min(idx_adc, N)

                for i in range(n_adc):
                    f.write("{},{},{},{}\n".format(times[i], data_adc[i],iman_pulse[i],periodo_us[i]))

            print("Guardado en la Pico: datos_motor.csv")
            print("Filas ADC:", n_adc, "Pulsos:", count)

        except KeyboardInterrupt:
            print("Guardado interrumpido por usuario")

        except Exception as e:
            print("Error al guardar:", e)
