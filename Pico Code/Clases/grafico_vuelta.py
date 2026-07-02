import serial
import numpy as np
import matplotlib.pyplot as plt

# ===========================
# CONFIG
# ===========================

ser = serial.Serial(
    "COM7",
    115200,
    timeout=1
)

plt.ion()

fig, ax = plt.subplots(figsize=(8,5))

linea, = ax.plot([], [], '.')

ax.set_xlabel("Theta [rad]")
ax.set_ylabel("Voltaje [V]")
ax.grid()

# ===========================
# DATOS
# ===========================

t_vuelta = []
V_vuelta = []

vuelta_actual = 0

# ===========================
# LOOP
# ===========================

while True:

    linea_serial = ser.readline().decode().strip()

    if not linea_serial:
        continue

    try:

        t, V, n = linea_serial.split(",")

        t = float(t)
        V = float(V)
        n = int(n)

        if n == vuelta_actual:

            t_vuelta.append(t)
            V_vuelta.append(V)

        else:

            if len(t_vuelta) > 5:

                t_np = np.array(t_vuelta)
                V_np = np.array(V_vuelta)

                theta = (
                    2*np.pi
                    * (t_np - t_np.min())
                    / (t_np.max() - t_np.min())
                )

                linea.set_xdata(theta)
                linea.set_ydata(V_np)

                ax.relim()
                ax.autoscale_view()

                plt.draw()
                plt.pause(0.001)

            vuelta_actual = n
            t_vuelta = [t]
            V_vuelta = [V]

    except Exception:
        pass