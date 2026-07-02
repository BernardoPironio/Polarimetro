from x9cxxx import X9Cxxx
from time import sleep
from machine import Pin, ADC, PWM

# Pines según tu conexión:
# GP14 -> INC
# GP13 -> U/D
# GP15 -> CS

motor = PWM(Pin(2))
motor.freq(25000)
motor.duty_u16(32768)


fot = ADC(0)

pot = X9Cxxx(inc_pin=14, ud_pin=13, cs_pin=15)


pot.set(99)
position = pot.get()
print(position)
