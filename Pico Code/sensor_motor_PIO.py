from machine import Pin, PWM, I2C, Timer # Pin: manage GPIO, PWM: generate PWM, I2C: read I2C, Timer: execute periodicly
from math import sin, pi # sin: sin function to generate table, pi: pi value to generate table 
from array import array
import micropython # micropython: special functions from runtime
import time
import gc # gc: garbage collector

# =========================================================
# RECOMENDADO PARA CALLBACKS / ISR
# =========================================================
micropython.alloc_emergency_exception_buf(100) # Protection for debug

# =========================================================
# I2C
# =========================================================
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=10000) # 0: bus 0, sda:SDA in pin 0, scl: SCL in pin 1, frec: frecuency
print("Devices found:", i2c.scan())

ADDR = 0x36 # Direction
STATUS = 0x0B # Sensor registry to read
i2c_buf = bytearray(1)   # fixed buffer to prevent allocs

# =========================================================
# PINES MOTOR
# =========================================================
PIN_U = 2 # fase 1
PIN_V = 3 # fase 2
PIN_W = 4 # fase 3
PIN_EN = 5 # Enable

# =========================================================
# PARAMETROS
# =========================================================
PWM_FREQ = 100000 # frecuencia portadora PWM
V_SUPPLY = 24.0
V_AMPLITUDE = 6.0

TABLE_SIZE = 360

UPDATE_HZ = 5500 # Frecuencia de actualización de conmutación(cuántas veces por segundo cambiás el duty)

STEP = 5 # Paso del índice por cada tick del timer

# =========================================================
# PWM HARDWARE
# =========================================================
pwm_u = PWM(Pin(PIN_U))
pwm_v = PWM(Pin(PIN_V))
pwm_w = PWM(Pin(PIN_W))

pwm_u.freq(PWM_FREQ)
pwm_v.freq(PWM_FREQ)
pwm_w.freq(PWM_FREQ)

en = Pin(PIN_EN, Pin.OUT) # Enable variable
en.value(0) # Disable

# =========================================================
# TABLAS PRECALCULADAS
# =========================================================
HALF_SUPPLY = V_SUPPLY / 2.0
SCALE = 65535.0 / V_SUPPLY # Volts to bits

lut_u = array('H', [0] * TABLE_SIZE) # array with [0, 0, 0, ..., 0] initially
lut_v = array('H', [0] * TABLE_SIZE)
lut_w = array('H', [0] * TABLE_SIZE)

for i in range(TABLE_SIZE):
    th = 2.0 * pi * i / TABLE_SIZE #theta values: th(i = 0) = 0, th(i = 90) = pi/2, th(i = 270) = 2pi/3

    ua = V_AMPLITUDE * sin(th) # Sinewave for fase 1
    ub = V_AMPLITUDE * sin(th - 2.0 * pi / 3.0) # Sinewave for fase 2
    uc = V_AMPLITUDE * sin(th - 4.0 * pi / 3.0) # Sinewave for fase 3

    ua_pwm = ua + HALF_SUPPLY # Positive placement
    ub_pwm = ub + HALF_SUPPLY # Positive placement
    uc_pwm = uc + HALF_SUPPLY # Positive placement

    # clamp(for protection)
    if ua_pwm < 0.0:
        ua_pwm = 0.0
    elif ua_pwm > V_SUPPLY:
        ua_pwm = V_SUPPLY

    if ub_pwm < 0.0:
        ub_pwm = 0.0
    elif ub_pwm > V_SUPPLY:
        ub_pwm = V_SUPPLY

    if uc_pwm < 0.0:
        uc_pwm = 0.0
    elif uc_pwm > V_SUPPLY:
        uc_pwm = V_SUPPLY

    lut_u[i] = int(ua_pwm * SCALE)
    lut_v[i] = int(ub_pwm * SCALE)
    lut_w[i] = int(uc_pwm * SCALE)

# =========================================================
# Motor global state
# =========================================================
phase_idx = 0 # initial index of the table
motor_running = False

# =========================================================
# CALLBACK DEL TIMER
# IMPORTANTE:
# - corto
# - sin prints
# - sin floats
# - sin crear objetos
# =========================================================
def motor_tick(timer):
    global phase_idx

    if not motor_running:
        return

    idx = phase_idx

    pwm_u.duty_u16(lut_u[idx])
    pwm_v.duty_u16(lut_v[idx])
    pwm_w.duty_u16(lut_w[idx])

    idx += STEP
    if idx >= TABLE_SIZE:
        idx -= TABLE_SIZE

    phase_idx = idx

# =========================================================
# TIMER PERIODICO
# =========================================================
tim = Timer() # create timer

# =========================================================
# MAIN
# =========================================================
count = 0 # lap count
md_prev = 0 # magnet state previous
t_antes = time.ticks_ms()
last_debug = time.ticks_ms()

try:
    gc.collect() # clean memory
    gc.disable() # disable garbage collector

    en.value(1) # enable driver
    motor_running = True 

    # hard=True -> menos jitter
    tim.init(freq=UPDATE_HZ, mode=Timer.PERIODIC, callback=motor_tick, hard=True)

    print("Motor encendido con PWM hardware + Timer")
    print("UPDATE_HZ =", UPDATE_HZ, "STEP =", STEP)

    while True:
        # lectura I2C fuera del camino crítico del motor
        i2c.readfrom_mem_into(ADDR, STATUS, i2c_buf)
        s = i2c_buf[0]
        md = (s >> 5) & 1

        if md == 1 and md_prev == 0:
            count += 1
            t_ms = time.ticks_ms()
            periodo = time.ticks_diff(t_ms, t_antes)
            t_antes = t_ms
            # no imprimir en cada flanco

        md_prev = md

        # debug ocasional
        now = time.ticks_ms()
        if time.ticks_diff(now, last_debug) > 1000:
            last_debug = now
            print("pulsos:", count, "mem_free:", gc.mem_free(), "phase_idx:", phase_idx)

except KeyboardInterrupt:
    print("\nInterrupción por teclado")

finally:
    motor_running = False
    tim.deinit()
    gc.enable()

    pwm_u.duty_u16(0)
    pwm_v.duty_u16(0)
    pwm_w.duty_u16(0)
    en.value(0)

    print("Motor apagado")
    print("Pulsos detectados:", count)