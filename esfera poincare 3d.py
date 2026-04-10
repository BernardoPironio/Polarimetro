#%%

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.optimize import curve_fit
from scipy.signal import find_peaks

#%%
#defino arrays de las mediciones

#orthogonal
medicion = pd.read_csv('Measurements\\medicion_manual_polarimetro_ortogonal.csv')
I_med_orth= medicion['voltaje']-0.00194
theta_orth = medicion['angulo']*np.pi/180
medicion.head()

#we calculate the first minimum
min_idx = None

for i in range(1, len(I_med_orth)-1):
    if I_med_orth[i] < I_med_orth[i-1] and I_med_orth[i] < I_med_orth[i+1]:
        min_idx = i
        break

#finally 
theta_orth = theta_orth[min_idx:].reset_index(drop=True)
I_med_orth = I_med_orth[min_idx:].reset_index(drop=True)

#--- estimate S0 --- and normalize voltaje 
# # --- Identify peaks in the voltage data to select the mean as the normalization factor --- 
peaks_volt = find_peaks(I_med_orth)[0] 
peaks_angle = theta_orth[peaks_volt] 
norm_number = I_med_orth[peaks_volt].mean() 

I_med_orth = I_med_orth / norm_number
S0_orth = np.max(I_med_orth)


#paralel
medicion = pd.read_csv('Measurements\\medicion_manual_polarimetro_paralelo.csv')
I_med_para = medicion['voltaje']-0.00194
theta_para = medicion['angulo']*np.pi/180
medicion.head()

#we plot the data to see if it looks like a sinusoidal function, as we expect
plt.plot(theta_para, I_med_para, 'o-')
plt.xlabel('Ángulo (rad)') 
plt.ylabel('Voltaje (V)')
plt.title('Voltaje vs Ángulo')
plt.grid()
plt.show()

# --- estimate S0 --- and normalize voltaje
# --- Identify peaks in the voltage data to select the mean as the normalization factor ---
peaks_volt = find_peaks(medicion['voltaje'])[0]
peaks_angle = theta_para[peaks_volt]
norm_number = I_med_para[peaks_volt].mean()


I_med_para = I_med_para / norm_number
S0_para = np.max(I_med_para)




#%%


# same model we used before
def delta_por_cuadrante(theta, d1, d2, d3, d4):
    theta = theta % (2*np.pi)
    
    if 0 <= theta < np.pi/2:
        return d1
    elif np.pi/2 <= theta < np.pi:
        return d2
    elif np.pi <= theta < 3*np.pi/2:
        return d3
    else:
        return d4


def model(theta, S1, S2, S3, d1, d2, d3, d4):
    I = []
    
    for t in theta:
        delta = delta_por_cuadrante(t, d1, d2, d3, d4)
        
        a0  = S0_orth/2 + S1/4*(1 + np.cos(delta))
        a4c = S1/4*(1 - np.cos(delta))
        a4s = S2/4*(1 - np.cos(delta))
        a2  = -S3/2*np.sin(delta)
        
        I.append(
            a0
            + a2*np.sin(2*t)
            + a4c*np.cos(4*t)
            + a4s*np.sin(4*t)
        )
    
    return np.array(I)

#curve fit 

initial_guess = [
    0.1*S0_orth,
    0.1*S0_orth,
    0.1*S0_orth,
    np.pi/2, np.pi/2, np.pi/2, np.pi/2
]

lower_bounds = [-S0_orth, -S0_orth, -S0_orth, 0, 0, 0, 0]
upper_bounds = [ S0_orth,  S0_orth,  S0_orth, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi]


#ajuste de orthogonal 

popt, pcov = curve_fit(
    model,
    theta_orth,
    I_med_orth,
    p0=initial_guess,
    sigma=I_err_orth,
    absolute_sigma=True,
    bounds=(lower_bounds, upper_bounds),maxfev=20000
)

#parameters
S1_fit, S2_fit, S3_fit, d1_fit, d2_fit, d3_fit, d4_fit = popt

param_errors = np.sqrt(np.diag(pcov))
S1_err, S2_err, S3_err, d1_err, d2_err, d3_err, d4_err = param_errors


# results
print("S0_orth =", S0_orth)
print(f"S1 = {S1_fit} ± {S1_err}")
print(f"S2 = {S2_fit} ± {S2_err}")
print(f"S3 = {S3_fit} ± {S3_err}")

print("\nDeltas:")
print(f"d1 = {d1_fit} ± {d1_err}")
print(f"d2 = {d2_fit} ± {d2_err}")
print(f"d3 = {d3_fit} ± {d3_err}")
print(f"d4 = {d4_fit} ± {d4_err}")


# DOP
DOP = np.sqrt(S1_fit**2 + S2_fit**2 + S3_fit**2) / S0_orth

DOP_err = (1 / S0_orth) * np.sqrt(
    (S1_fit * S1_err)**2 +
    (S2_fit * S2_err)**2 +
    (S3_fit * S3_err)**2
) / np.sqrt(S1_fit**2 + S2_fit**2 + S3_fit**2)

print(f"\nDOP = {DOP} ± {DOP_err}")


#Curve for graph
theta_fit = np.linspace(theta_orth.min(), theta_orth.max(), 500)
I_fit = model(theta_fit, *popt)


# graph
plt.errorbar(theta_orth, I_med_orth, yerr=I_err_orth, xerr=theta_err_orth,
             fmt='o', capsize=3, label="Datos")

plt.plot(theta_fit, I_fit, color="orange", label="Ajuste curve_fit")

plt.grid()
plt.xlabel("Theta (rad)")
plt.ylabel("Intensidad")
plt.legend()
plt.show()

#%%


#ajuste de parallels


# same model we used before
def delta_por_cuadrante(theta, d1, d2, d3, d4):
    theta = theta % (2*np.pi)
    
    if 0 <= theta < np.pi/2:
        return d1
    elif np.pi/2 <= theta < np.pi:
        return d2
    elif np.pi <= theta < 3*np.pi/2:
        return d3
    else:
        return d4


def model(theta, S1, S2, S3, d1, d2, d3, d4):
    I = []
    
    for t in theta:
        delta = delta_por_cuadrante(t, d1, d2, d3, d4)
        
        a0  = S0_para/2 + S1/4*(1 + np.cos(delta))
        a4c = S1/4*(1 - np.cos(delta))
        a4s = S2/4*(1 - np.cos(delta))
        a2  = -S3/2*np.sin(delta)
        
        I.append(
            a0
            + a2*np.sin(2*t)
            + a4c*np.cos(4*t)
            + a4s*np.sin(4*t)
        )
    
    return np.array(I)

#curve fit 

initial_guess = [
    0.1*S0_para,
    0.1*S0_para,
    0.1*S0_para,
    np.pi/2, np.pi/2, np.pi/2, np.pi/2
]

lower_bounds = [-S0_para, -S0_para, -S0_para, 0, 0, 0, 0]
upper_bounds = [ S0_para,  S0_para,  S0_para, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi]

popt, pcov = curve_fit(
    model,
    theta_para,
    I_med_para,
    p0=initial_guess,
    sigma=I_err_para,
    absolute_sigma=True,
    bounds=(lower_bounds, upper_bounds),maxfev=20000
)

#parameters
S1_fit, S2_fit, S3_fit, d1_fit, d2_fit, d3_fit, d4_fit = popt

param_errors = np.sqrt(np.diag(pcov))
S1_err, S2_err, S3_err, d1_err, d2_err, d3_err, d4_err = param_errors


# results
print("S0_orth =", S0_para)
print(f"S1 = {S1_fit} ± {S1_err}")
print(f"S2 = {S2_fit} ± {S2_err}")
print(f"S3 = {S3_fit} ± {S3_err}")

print("\nDeltas:")
print(f"d1 = {d1_fit} ± {d1_err}")
print(f"d2 = {d2_fit} ± {d2_err}")
print(f"d3 = {d3_fit} ± {d3_err}")
print(f"d4 = {d4_fit} ± {d4_err}")


# DOP
DOP = np.sqrt(S1_fit**2 + S2_fit**2 + S3_fit**2) / S0_para

DOP_err = (1 / S0_para) * np.sqrt(
    (S1_fit * S1_err)**2 +
    (S2_fit * S2_err)**2 +
    (S3_fit * S3_err)**2
) / np.sqrt(S1_fit**2 + S2_fit**2 + S3_fit**2)

print(f"\nDOP = {DOP} ± {DOP_err}")


#Curve for graph
theta_fit = np.linspace(theta_para.min(), theta_para.max(), 500)
I_fit = model(theta_fit, *popt)


# graph
plt.errorbar(theta_para, I_med_para, yerr=I_err_para, xerr=theta_err_para,
             fmt='o', capsize=3, label="Datos")

plt.plot(theta_fit, I_fit, color="orange", label="Ajuste curve_fit")

plt.grid()
plt.xlabel("Theta (rad)")
plt.ylabel("Intensidad")
plt.legend()
plt.show()


#%%

#poincare vectors

#ORTHOGONAL
# --- vector de Stokes normalizado (Poincaré) ---
s1_orth = S1_fitO / S0_orth
s2_orth = S2_fitO / S0_orth
s3_orth = S3_fitO / S0_orth

# --- errores ---
s1_err_orth = S1_errO / S0_orth
s2_err_orth = S2_errO / S0_orth
s3_err_orth = S3_errO / S0_orth

print("\nVector de Poincaré:")
print(f"s1 = {s1_orth} ± {s1_err_orth}")
print(f"s2 = {s2_orth} ± {s2_err_orth}")
print(f"s3 = {s3_orth} ± {s3_err_orth}")


s_curf_orth=np.array([s1_orth,s2_orth,s3_orth])

# --- vector de Stokes normalizado (Poincaré) ---
s1_para = S1_fit / S0_para
s2_para = S2_fit / S0_para
s3_para = S3_fit / S0_para

# --- errores ---
s1_err_para = S1_err / S0_para
s2_err_para = S2_err / S0_para
s3_err_para = S3_err / S0_para

print("\nVector de Poincaré:")
print(f"s1 = {s1_para} ± {s1_err_para}")
print(f"s2 = {s2_para} ± {s2_err_para}")
print(f"s3 = {s3_para} ± {s3_err_para}")

s_curf_para=np.array([s1_para,s2_para,s3_para])

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
s2=s_curf_para
s1=s_curf_orth
plot_poincare_sphere(ax, s1,s2)

plt.show()