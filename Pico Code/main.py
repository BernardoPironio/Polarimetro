from machine import Pin, PWM, I2C, Timer, ADC
from math import sin, pi
from array import array
import micropython
import time
import gc

# Protección para debug en interrupciones
micropython.alloc_emergency_exception_buf(100)

# =========================================================
# 1. DEFINICIÓN DE PERIFÉRICOS (Todo arriba)
# =========================================================
fotodiodo = ADC(Pin(26))

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=10000)
ADDR = 0x36 
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
V_SUPPLY = 24.0
V_AMPLITUDE = 6.0
TABLE_SIZE = 360
UPDATE_HZ = 5500 
STEP = 7 

HALF_SUPPLY = V_SUPPLY / 2.0
SCALE = 65535.0 / V_SUPPLY

lut_u = array('H', [0] * TABLE_SIZE)
lut_v = array('H', [0] * TABLE_SIZE)
lut_w = array('H', [0] * TABLE_SIZE)

for i in range(TABLE_SIZE):
    th = 2.0 * pi * i / TABLE_SIZE
    ua = V_AMPLITUDE * sin(th) + HALF_SUPPLY
    ub = V_AMPLITUDE * sin(th - 2.0 * pi / 3.0) + HALF_SUPPLY
    uc = V_AMPLITUDE * sin(th - 4.0 * pi / 3.0) + HALF_SUPPLY
    
    lut_u[i] = int(max(0, min(V_SUPPLY, ua)) * SCALE)
    lut_v[i] = int(max(0, min(V_SUPPLY, ub)) * SCALE)
    lut_w[i] = int(max(0, min(V_SUPPLY, uc)) * SCALE)

# Estado global
phase_idx = 0
periodos = array('I', [0] * 1000) # Definido fuera del try

# =========================================================
# 3. CALLBACK DEL TIMER (Optimizado)
# =========================================================
def motor_tick(timer):
    global phase_idx
    # Usamos una variable local para velocidad
    idx = phase_idx
    
    pwm_u.duty_u16(lut_u[idx])
    pwm_v.duty_u16(lut_v[idx])
    pwm_w.duty_u16(lut_w[idx])

    # Avanzar índice con módulo (más seguro)
    phase_idx = (idx + STEP) % TABLE_SIZE

# =========================================================
# 4. MAIN
# =========================================================
tim = Timer()
count = 0
md_prev = 0

N = 5000
t0 = time.ticks_us()
data_adc = array('H',[0]*N)
times = array('I',[0]*N)

idx_adc = 0

t_antes = time.ticks_ms()
last_debug = time.ticks_ms()

try:
    gc.collect()
    # No desactivamos GC completamente para evitar NameErrors por falta de RAM
    
    en.value(1)
    phase_idx = 0
    
    print("Iniciando motor...")
    # El Timer se inicia AL FINAL de las preparaciones
    tim.init(freq=UPDATE_HZ, mode=Timer.PERIODIC, callback=motor_tick, hard=True)
    

    while True:
        i2c.readfrom_mem_into(ADDR, STATUS, i2c_buf)
        md = (i2c_buf[0] >> 5) & 1
        
        if md == 1 and md_prev == 0:
            t_ms = time.ticks_ms()
            periodo = time.ticks_diff(t_ms, t_antes)
            
            # Protección de índice para el array periodos
            periodos[count % 1000] = periodo
            count += 1
            t_antes = t_ms

        md_prev = md
        
        #fotodiodo
        times[idx_adc] = time.ticks_diff(time.ticks_us(),t0)
        data_adc[idx_adc] = fotodiodo.read_u16()
        
        idx_adc += 1

        # Debug cada 1 segundo
        now = time.ticks_ms()
        if time.ticks_diff(now, last_debug) > 1000:
            last_debug = now
            print("Pulsos:", count, "Mem:", gc.mem_free(), "Fase:", phase_idx,'Intensidad:', data_adc[idx_adc-1])

except KeyboardInterrupt:
    print("\nDetenido por usuario")

finally:
    tim.deinit()
    en.value(0)
    pwm_u.duty_u16(0)
    pwm_v.duty_u16(0)
    pwm_w.duty_u16(0)
    print("Sistema apagado. Pulsos totales:", count)
    
    # Al final puedes ver los periodos guardados
    
    print('Periodos: ',periodos)
    print('\n')
    
    print('Times: ', times)
    print('\n')
    
    print('ADC raw: ',data_adc)
    
    print(len(data_adc),len(times))
            
            
            
            
            
            
            