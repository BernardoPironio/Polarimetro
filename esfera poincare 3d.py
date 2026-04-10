#%%

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.optimize import curve_fit
from scipy.signal import find_peaks

#%%
#vectores de poincare
s_orth=np.array([-0.81525447,  0.62860321, -0.10498477])
s_paral=np.array([ 0.91821774 ,-0.14678917 ,-0.01286342])


#%%

def plot_poincare_sphere(ax, s1, s2):
    u = np.linspace(0, 2 * np.pi, 80)
    v = np.linspace(0, np.pi, 40)

    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))

    ax.plot_wireframe(x, y, z, rstride=4, cstride=4, alpha=0.12)

    # ejes
    ax.plot([-1, 1], [0, 0], [0, 0], lw=1)
    ax.plot([0, 0], [-1, 1], [0, 0], lw=1)
    ax.plot([0, 0], [0, 0], [-1, 1], lw=1)

    # --- vector 1 ---
    ax.scatter(s1[0], s1[1], s1[2], s=90, label="Orthogonal")
    ax.plot([0, s1[0]], [0, s1[1]], [0, s1[2]], "--", lw=2)

    # --- vector 2 ---
    ax.scatter(s2[0], s2[1], s2[2], s=90, label="Parallel")
    ax.plot([0, s2[0]], [0, s2[1]], [0, s2[2]], "--", lw=2)

    ax.set_xlabel("s1")
    ax.set_ylabel("s2")
    ax.set_zlabel("s3")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_box_aspect((1, 1, 1))
    ax.set_title("Poincare sphere")
    ax.legend()

    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
s2=s_orth
s1=s_paral
plot_poincare_sphere(ax, s1,s2)

plt.show()
# %%
