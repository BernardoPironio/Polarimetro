from machine import Pin, PWM, I2C
from math import sin, pi
import time

# -----------------------------
# I2C
# -----------------------------
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=10000)
print("Devices found:", i2c.scan())

ADDR = 0x36
STATUS = 0x0B

# -----------------------------
# PINES MOTOR
# -----------------------------
PIN_U = 2
PIN_V = 3
PIN_W = 4
PIN_EN = 5

# -----------------------------
# PARAMETROS
# -----------------------------
PWM_FREQ = 100000
V_SUPPLY = 24.0
V_AMPLITUDE = 6.0
VELOCIDAD = 300.0

# -----------------------------
# PWM
# -----------------------------
pwm_u = PWM(Pin(PIN_U))
pwm_v = PWM(Pin(PIN_V))
pwm_w = PWM(Pin(PIN_W))

pwm_u.freq(PWM_FREQ)
pwm_v.freq(PWM_FREQ)
pwm_w.freq(PWM_FREQ)

en = Pin(PIN_EN, Pin.OUT)
en.value(0)   # arranca apagado

# -----------------------------
# FUNCION AUXILIAR
# -----------------------------
def set_phase_voltage(pwm_obj, voltage, v_supply):
    if voltage < 0:
        voltage = 0
    if voltage > v_supply:
        voltage = v_supply

    duty = int((voltage / v_supply) * 65535)
    pwm_obj.duty_u16(duty)

# -----------------------------
# VARIABLES
# -----------------------------
theta = 0.0
t_prev = time.ticks_us()
count = 0
md_prev = 0   # para detectar flanco

# -----------------------------
# MAIN
# -----------------------------



try:
    en.value(1)   # habilitar driver

    while True:
        
        
        
        
        # leer sensor
        s = i2c.readfrom_mem(ADDR, STATUS, 1)[0]
        md = (s >> 5) & 1

        # detectar flanco ascendente: 0 -> 1
        if md == 1 and md_prev == 0:
            #print("TIC", count)
            count += 1

        md_prev = md

        # tiempo transcurrido
        t_now = time.ticks_us()
        dt = time.ticks_diff(t_now, t_prev) / 1_000_000.0
        t_prev = t_now

        # avanzar ángulo eléctrico
        theta += VELOCIDAD * dt

        # tres senoides desfasadas 120°
        ua = V_AMPLITUDE * sin(theta)
        ub = V_AMPLITUDE * sin(theta - 2*pi/3)
        uc = V_AMPLITUDE * sin(theta - 4*pi/3)

        # desplazar al rango 0 ... V_SUPPLY
        ua_pwm = ua + V_SUPPLY / 2
        ub_pwm = ub + V_SUPPLY / 2
        uc_pwm = uc + V_SUPPLY / 2

        # aplicar al driver
        set_phase_voltage(pwm_u, ua_pwm, V_SUPPLY)
        set_phase_voltage(pwm_v, ub_pwm, V_SUPPLY)
        set_phase_voltage(pwm_w, uc_pwm, V_SUPPLY)


finally:
    print("Apagando motor...")

    pwm_u.duty_u16(0)
    pwm_v.duty_u16(0)
    pwm_w.duty_u16(0)

    en.value(0)
    
    print(count)