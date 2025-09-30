import numpy as np
import matplotlib.pyplot as plt


theta_p = -1/3*np.pi
theta_q = -1/12*np.pi
theta_r = 0
theta_s = 1/12*np.pi
theta_t = 1/2*np.pi


a = np.array([1.2, -5.0, 30.0, -7.5, 0.75])
b = np.array([0.25, 0.1, 0.1, 0.1, 0.4])
theta_i = np.array([theta_p, theta_q, theta_r, theta_s, theta_t])

# Sampling
fs = 1000
T_max = 10
t = np.linspace(0, T_max, int(T_max*fs))
dt = t[1] - t[0]

omega = 2 * np.pi # angular frequency (rad/s)

theta = np.zeros_like(t)
z = np.zeros_like(t)


def wrap_phase(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


for i in range(1, len(t)):
    # Phase update
    dtheta = omega
    theta[i] = wrap_phase(theta[i-1] + dtheta * dt)

    noise = 0
    theta[i] = wrap_phase(theta[i-1] + omega * dt)
    delta_theta = wrap_phase(theta[i] - theta_i)  # vector of differences
    dz = -np.sum(a * delta_theta * np.exp(-delta_theta**2 / (2 * b**2)))
    z[i] = z[i-1] + dz * dt + noise

#z = 2 * (z - np.min(z)) / (np.max(z) - np.min(z)) - 1


# Normalize to ±1 mV
#z = z / np.max(np.abs(z)) * 1.0

# Plot
plt.figure(figsize=(12, 4))
plt.plot(t, z, label="ECG sintetico (Sameni)")
plt.xlabel("Tempo (s)")
plt.ylabel("Ampiezza (mV)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
