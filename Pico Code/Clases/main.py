from measurment_class import Medicion
from x9cxxx import X9Cxxx

pot = X9Cxxx(
    inc_pin=14,
    ud_pin=13,
    cs_pin=15
)

pot.set(0)

m = Medicion()


m.medir_continuo()

