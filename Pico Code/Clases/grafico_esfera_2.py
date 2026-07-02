import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (necesario para proj='3d')

from scipy.optimize import least_squares

# =========================================================
# FUNCIONES
# =========================================================

def delta_suave(theta, d1, d2, d3, d4):

    th = np.mod(theta, 2*np.pi)

    theta_nodes = np.array([
        0,
        np.pi/2,
        np.pi,
        3*np.pi/2,
        2*np.pi
    ])

    delta_nodes = np.array([
        d1,
        d2,
        d3,
        d4,
        d1
    ])

    return np.interp(
        th,
        theta_nodes,
        delta_nodes
    )


def model_angulo(
    theta,
    S0,
    S1,
    S2,
    S3,
    d1,
    d2,
    d3,
    d4,
    theta0,
):

    theta = theta + theta0

    delta = delta_suave(
        theta,
        d1,
        d2,
        d3,
        d4
    )

    a0 = (
        S0/2
        + S1/4*(1 + np.cos(delta))
    )

    a4c = (
        S1/4*(1 - np.cos(delta))
    )

    a4s = (
        S2/4*(1 - np.cos(delta))
    )

    a2 = (
        -S3/2*np.sin(delta)
    )

    return (
        a0
        + a2*np.sin(2*theta)
        + a4c*np.cos(4*theta)
        + a4s*np.sin(4*theta)
    )


def fit_una_vuelta_angulo(theta, I):

    # =========================================
    # ordenar
    # =========================================

    idx = np.argsort(theta)

    theta = theta[idx]
    I = I[idx]

    # =========================================
    # normalizar
    # =========================================

    I = I / np.max(I)

    S0 = np.max(I)

    # =========================================
    # errores
    # =========================================

    V_div = 0.005

    V_full_scale = 8 * V_div

    res_err = V_full_scale / 256

    I_err = np.sqrt(
        (0.03 * I)**2
        + res_err**2
    )

    # =========================================
    # residuals
    # =========================================

    def residuals(params):

        (
            S1,
            S2,
            S3,
            d1,
            d2,
            d3,
            d4,
            theta0
        ) = params

        I_model = model_angulo(
            theta,
            S0,
            S1,
            S2,
            S3,
            d1,
            d2,
            d3,
            d4,
            theta0
        )

        return (
            I_model - I
        ) / I_err

    # =========================================
    # bounds
    # =========================================

    lower_bounds = [
        -S0,
        -S0,
        -S0,
        0,
        0,
        0,
        0,
        -2*np.pi
    ]

    upper_bounds = [
        S0,
        S0,
        S0,
        2*np.pi,
        2*np.pi,
        2*np.pi,
        2*np.pi,
        2*np.pi
    ]

    # =========================================
    # multistart
    # =========================================

    best_result = None
    best_cost = np.inf

    for _ in range(20):

        guess = [

            np.random.uniform(-S0, S0),
            np.random.uniform(-S0, S0),
            np.random.uniform(-S0, S0),

            np.random.uniform(1.4, 1.9),
            np.random.uniform(1.4, 1.9),
            np.random.uniform(1.4, 1.9),
            np.random.uniform(1.4, 1.9),

            np.random.uniform(-np.pi, np.pi)
        ]

        r = least_squares(
            residuals,
            guess,
            bounds=(
                lower_bounds,
                upper_bounds
            )
        )

        if r.cost < best_cost:

            best_cost = r.cost
            best_result = r

    result = best_result

    (
        S1_fit,
        S2_fit,
        S3_fit,
        d1_fit,
        d2_fit,
        d3_fit,
        d4_fit,
        theta0_fit
    ) = result.x

    # =========================================
    # curva ajustada
    # =========================================

    theta_fit = np.linspace(
        0,
        2*np.pi,
        3000
    )

    I_fit = model_angulo(
        theta_fit,
        S0,
        S1_fit,
        S2_fit,
        S3_fit,
        d1_fit,
        d2_fit,
        d3_fit,
        d4_fit,
        theta0_fit
    )

    resultados = {

        "s1": S1_fit / S0,
        "s2": S2_fit / S0,
        "s3": S3_fit / S0,

        "theta_fit": theta_fit,
        "I_fit": I_fit
    }

    return resultados


# =========================================================
# CONFIG
# =========================================================

R = 2

theta_common = np.linspace(
    0,
    2*np.pi,
    500
)

# =========================================================
# SERIAL
# =========================================================

ser = serial.Serial(
    "COM7",
    115200,
    timeout=1
)

# =========================================================
# ARRAYS
# =========================================================

time_data = []
voltaje_data = []
vueltas_data = []

# historial de puntos en la esfera de Poincaré
s1_hist = []
s2_hist = []
s3_hist = []

# =========================================================
# PLOT 3D - ESFERA DE POINCARÉ
# =========================================================

plt.ion()

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')

# =========================================
# dibujar la esfera unidad (wireframe)
# =========================================

u = np.linspace(0, 2*np.pi, 40)
v = np.linspace(0, np.pi, 40)

x_sphere = np.outer(np.cos(u), np.sin(v))
y_sphere = np.outer(np.sin(u), np.sin(v))
z_sphere = np.outer(np.ones_like(u), np.cos(v))

ax.plot_wireframe(
    x_sphere,
    y_sphere,
    z_sphere,
    color='gray',
    alpha=0.2,
    linewidth=0.5
)

# ejes de referencia (S1, S2, S3)
ax.plot([-1, 1], [0, 0], [0, 0], color='k', lw=0.8)
ax.plot([0, 0], [-1, 1], [0, 0], color='k', lw=0.8)
ax.plot([0, 0], [0, 0], [-1, 1], color='k', lw=0.8)

ax.set_xlabel("S1")
ax.set_ylabel("S2")
ax.set_zlabel("S3")

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

ax.set_box_aspect([1, 1, 1])

ax.set_title("Trayectoria en la esfera de Poincaré")

# trayectoria acumulada (línea que une los puntos en orden)
linea_trayectoria, = ax.plot(
    [],
    [],
    [],
    '-',
    color='tab:blue',
    alpha=0.5,
    lw=1
)

# puntos ya graficados (quedan fijos)
puntos_previos = ax.scatter(
    [],
    [],
    [],
    color='tab:blue',
    s=20
)

# último punto (resaltado en otro color)
punto_actual = ax.scatter(
    [],
    [],
    [],
    color='red',
    s=60
)

# =========================================================
# CONTROL DE BLOQUES
# =========================================================

bloque_actual = 0

# =========================================================
# LOOP PRINCIPAL
# =========================================================

while True:

    linea = ser.readline().decode().strip()

    if linea:

        try:

            # =========================================
            # LEER SERIAL
            # =========================================

            t, V, n = linea.split(',')

            t = float(t)
            V = float(V)
            n = int(n)

            # =========================================
            # GUARDAR
            # =========================================

            time_data.append(t)
            voltaje_data.append(V)
            vueltas_data.append(n)

            # =========================================
            # ARRAYS NUMPY
            # =========================================

            time_np = np.array(time_data)
            voltaje_np = np.array(voltaje_data)
            vueltas_np = np.array(vueltas_data)

            vueltas_unicas = np.unique(
                vueltas_np
            )

            vueltas_necesarias = (
                (bloque_actual + 1)
                * R
            )

            # =========================================
            # ¿HAY BLOQUE COMPLETO?
            # =========================================

            if len(vueltas_unicas) >= vueltas_necesarias:

                lista = vueltas_unicas[
                    bloque_actual*R :
                    (bloque_actual+1)*R
                ]

                print(
                    f"\nProcesando vueltas "
                    f"{lista[0]} -> {lista[-1]}"
                )

                I_all = []

                # =========================================
                # PROCESAR CADA VUELTA
                # =========================================

                for vuelta in lista:

                    mask = (
                        vueltas_np == vuelta
                    )

                    t_vuelta = time_np[mask]
                    V_vuelta = voltaje_np[mask]

                    if len(t_vuelta) < 2:
                        continue

                    idx = np.argsort(
                        t_vuelta
                    )

                    t_vuelta = t_vuelta[idx]
                    V_vuelta = V_vuelta[idx]

                    dt = (
                        t_vuelta.max()
                        - t_vuelta.min()
                    )

                    if dt == 0:
                        continue

                    theta = (

                        2*np.pi

                        * (
                            t_vuelta
                            - t_vuelta.min()
                        )

                        / dt
                    )

                    I_interp = np.interp(
                        theta_common,
                        theta,
                        V_vuelta
                    )

                    I_all.append(
                        I_interp
                    )

                # =========================================
                # VERIFICAR
                # =========================================

                if len(I_all) == 0:

                    print("Bloque vacío")

                    continue

                # =========================================
                # PROMEDIO
                # =========================================

                I_all = np.array(I_all)

                I_mean = np.mean(
                    I_all,
                    axis=0
                )

                # =========================================
                # AJUSTE
                # =========================================

                resultados = fit_una_vuelta_angulo(
                    theta_common,
                    I_mean
                )

                s1 = resultados["s1"]
                s2 = resultados["s2"]
                s3 = resultados["s3"]

                print(
                    f"S1={s1:.3f}  S2={s2:.3f}  S3={s3:.3f}"
                )

                # =========================================
                # ACTUALIZAR HISTORIAL
                # =========================================

                s1_hist.append(s1)
                s2_hist.append(s2)
                s3_hist.append(s3)

                # =========================================
                # ACTUALIZAR PLOT 3D
                # =========================================

                # línea de trayectoria (todos los puntos, en orden)
                linea_trayectoria.set_data(
                    s1_hist,
                    s2_hist
                )
                linea_trayectoria.set_3d_properties(
                    s3_hist
                )

                # puntos previos (todos menos el último)
                if len(s1_hist) > 1:
                    puntos_previos._offsets3d = (
                        s1_hist[:-1],
                        s2_hist[:-1],
                        s3_hist[:-1]
                    )

                # último punto (resaltado)
                punto_actual._offsets3d = (
                    [s1_hist[-1]],
                    [s2_hist[-1]],
                    [s3_hist[-1]]
                )

                plt.draw()
                plt.pause(0.01)

                print("Punto graficado en la esfera")

                # =========================================
                # SIGUIENTE BLOQUE
                # =========================================

                bloque_actual += 1

        except Exception as e:

            print(e)