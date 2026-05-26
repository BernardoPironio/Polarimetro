from machine import Pin, ADC
import time

cs = Pin(15,Pin.OUT)
inc = Pin(14,Pin.OUT)
ud = Pin(13,Pin.OUT)


cs.value(1)
inc.value(1)
ud.value(1)


while True:
    inc.value(0)
    time.sleep(.1)
    inc.value(1)
    time.sleep(0.5)


