from x9cxxx import X9Cxxx
from time import sleep

# Pines según tu conexión:
# GP14 -> INC
# GP13 -> U/D
# GP15 -> CS

pot = X9Cxxx(inc_pin=14, ud_pin=13, cs_pin=15)


pot.set(50)
position = pot.get()
print(position)