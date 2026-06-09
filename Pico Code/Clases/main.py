from measurment_class import Medicion
from x9cxxx import X9Cxxx

pot = X9Cxxx(
    inc_pin=14,
    ud_pin=13,
    cs_pin=15
)

pot.set(0)

m = Medicion()

resultado = m.calibrar(
    delta=0.08,
    vueltas=3
)

if not resultado["satura"]:

    m.medir_continuo()

else:

    print(
        "No se encontró posición válida."
    )