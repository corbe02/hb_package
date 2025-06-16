"""
A Dynamical model for generarting synthetic electrocardiograms signamls (McSharry, Clifford...)

Idea:
Il modello genera una traiettoria in 3 dimensioni con coordinate x,y,z. La quasi periodicià è raggiunta impinendo il movimento 
della traiettoria attorno ad un cerchio di raggio unitario nel piano x,y.
Vari punti dell'ecg (PQRST) sono descritti da eventi che corrispondono a espansioni o riduzioni sull' assse z.
Ogni punto di PQRST è legato ad uno specifico angolo del cerchio unitario (theta_i).
Quando la traiettoria raggiunge uno di questi angoli, spinge o tira z verso il il cerchio 

"""


import numpy as np
import matplotlib.pyplot as plt

theta_p = -1/3*np.pi
theta_q =  -1/12*np.pi
theta_r = 0
theta_s = 1/12*np.pi
theta_t = 1/2*np.pi

a_p = 1.2
a_q =  -5.0
a_r = 30.0
a_s = -7.5
a_t = 0.75

b_p = 0.25
b_q =  0.1
b_r = 0.1
b_s = 0.1
b_t = 0.4


theta_i = np.array([theta_p, theta_q, theta_r, theta_s, theta_t])
a = np.array([a_p,a_q, a_r, a_s, a_t])
b = np.array([b_p, b_q, b_r, b_s, b_t])

fs = 1000 #frequenza di campionamento
T_max = 10 #secondi

t = np.linspace(0, T_max, int(T_max*fs))
x = np.zeros_like(t)
y = np.zeros_like(t)
theta = np.zeros_like(t)
z = np.zeros_like(t)

#Inizio
x[0] = 0.0
y[0] = 1.0

alpha = 1 - np.sqrt(x[0]**2+y[0]**2)
omega = 2*np.pi # 1Hz --> velocità angolare
f2 = 0.2 #Hz
A = 0.00015

dt = t[1]-t[0]

for i in range(1, len(t)):
    dx = alpha*x[i-1] - omega*y[i-1]
    dy = alpha*y[i-1] + omega*x[i-1]

    x[i] = x[i-1]+ dx*dt
    y[i] = y[i-1]+ dy*dt

    theta[i] = np.arctan2(y[i],x[i])
    alpha = 1 - np.sqrt(x[i]**2+y[i]**2)

    z0 = A * np.sin(2*np.pi*f2*t[i])

    dz_term = 0.0
    for j in range(len(theta_i)):
        delta_theta = (theta[i] - theta_i[j] + np.pi) % (2*np.pi) - np.pi
        dz_term += a[j] * delta_theta * np.exp(-delta_theta**2 / (2 * b[j]**2))

    dz = -dz_term - (z[i-1] - z0)
    z[i] = z[i-1] + dz * dt
    #z = -z

z = z / np.max(np.abs(z)) * 1.0  # per normalizzare il picco massimo a 1 mV


plt.figure(figsize=(12,4))
plt.plot(t, z, label="ECG sintetico")
plt.xlabel("Tempo (s)")
plt.ylabel("Ampiezza z (mV)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()



