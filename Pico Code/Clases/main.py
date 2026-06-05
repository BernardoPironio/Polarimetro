from measurment_class import Medicion

m = Medicion(resistencia = 99)

resistencia = m.calibrar_resistencia()

m = Medicion(resistencia = resistencia)

try:
    datos = m.medir(vueltas = 16, muestras_por_vuelta = 600)
    m.guardar_csv(datos)
except:
    print('Probablemente se quedo sin memoria, reduzca las muetstras por vuelta o las vueltas')

