from machine import Pin,ADC
from time import sleep

led = Pin('LED',Pin.OUT)
sensor_binary = Pin(26,Pin.IN)
#sensor = ADC(Pin(26))

while True:
    
    #print(sensor.read_u16())
    print(sensor_binary.value())
    
    #if sensor.read_u16() > 1000:
    if sensor_binary.value() == 1:
        
        led.value(1)
    else:
        led.value(0)
        
    sleep(.1)
    

