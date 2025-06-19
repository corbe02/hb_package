# #!/usr/bin/env python3
# import rospy
# import neurokit2 as nk
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float32
# import time
# import numpy as np

# def ecg_node():
#     rospy.init_node('ecg_simulator_plotter', anonymous=True)
#     ecg_pub = rospy.Publisher('ecg_signal', Float32, queue_size=10)

#     try:
#         duration = 5  # secondi
#         sampling_rate = 60 # Hz
#         ecg = nk.ecg_simulate(duration=duration, sampling_rate=sampling_rate, method="ecgsyn")
#         t = np.linspace(0, duration, len(ecg))

#         mean = np.mean(ecg)
#         std = np.std(ecg)
#         print(mean)
#         print(std)

#         noise_std = 0.05 
#         noise = np.random.normal(loc=0.0, scale=noise_std, size=len(ecg))
#         noisy_heartbeat = ecg + noise

#         # # NOISE ADDITION
#         # # 1. Low-frequency drift
#         # drift = 0.2 * np.sin(2 * np.pi * 0.5 * t)

#         # # 2. Laplace noise
#         # laplace_noise = np.random.laplace(loc=0.0, scale=0.1, size=t.shape)

#         # # 3. Impulse noise
#         # impulses = np.zeros_like(t)
#         # impulse_indices = np.random.choice(len(t), size=10, replace=False)
#         # impulses[impulse_indices] = np.random.uniform(-1, 1, size=10)

#         # # 4. Multiplicative noise
#         # multiplicative_envelope = 1 + 0.1 * np.random.randn(*t.shape)

#         # # Final noisy signal
#         # noisy_heartbeat = (ecg + drift + laplace_noise + impulses) * multiplicative_envelope

#         plt.figure(figsize=(12, 5))
#         plt.plot(t, ecg, label="ECG originale", alpha=0.7)
#         plt.plot(t, noisy_heartbeat, label="ECG con rumore", alpha=0.7)
#         plt.title("Confronto tra ECG originale e con rumore")
#         plt.xlabel("Tempo (s)")
#         plt.ylabel("Ampiezza")
#         plt.legend()
#         plt.grid(True)
#         plt.tight_layout()
#         plt.show()

#         # Pubblicazione su ROS
#         rate = 60
#         sleep_time = 1.0 / rate
#         rospy.loginfo(f"Lunghezza del segnale ECG: {len(ecg)}, Frequenza di pubblicazione: {rate} Hz")

#         for i, sample in enumerate(noisy_heartbeat):
#             if rospy.is_shutdown():
#                 rospy.loginfo("Ricevuto segnale di shutdown, terminando la pubblicazione.")
#                 break
#             ecg_msg = Float32()
#             ecg_msg.data = sample
#             ecg_pub.publish(ecg_msg)
#             rospy.loginfo(f"Pubblicato campione {i}: {sample}")
#             time.sleep(sleep_time)

#         rospy.loginfo("Pubblicazione del segnale ECG completata.")
        
#     except Exception as e:
#         rospy.logerr(f"Si è verificato un errore: {e}")

# if __name__ == '__main__':
#     try:
#         ecg_node()
#     except Exception as e:
#         rospy.logerr(f"Si è verificato un errore: {e}")

#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt

def generate_ecg_signal(T_max=10, fs=60):
    """Genera il segnale ECG sintetico usando il modello di McSharry."""
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
    a = np.array([a_p, a_q, a_r, a_s, a_t])
    b = np.array([b_p, b_q, b_r, b_s, b_t])

    t = np.linspace(0, T_max, int(T_max*fs))
    x = np.zeros_like(t)
    y = np.zeros_like(t)
    theta = np.zeros_like(t)
    z = np.zeros_like(t)

    x[0] = 0.0
    y[0] = 1.0

    alpha = 1 - np.sqrt(x[0]**2 + y[0]**2)
    omega = 2*np.pi  # 1 Hz (frequenza cardiaca)
    dt = t[1] - t[0]

    for i in range(1, len(t)):
        dx = alpha * x[i-1] - omega * y[i-1]
        dy = alpha * y[i-1] + omega * x[i-1]

        x[i] = x[i-1] + dx * dt
        y[i] = y[i-1] + dy * dt

        theta[i] = np.arctan2(y[i], x[i])
        alpha = 1 - np.sqrt(x[i]**2 + y[i]**2)

        dz_term = 0.0
        for j in range(len(theta_i)):
            delta_theta = (theta[i] - theta_i[j] + np.pi) % (2 * np.pi) - np.pi
            dz_term += a[j] * delta_theta * np.exp(-delta_theta**2 / (2 * b[j]**2))

        dz = -dz_term
        z[i] = z[i-1] + dz * dt

    # Normalizza l’ECG al range [-1,1]
    z = z / np.max(np.abs(z))

    return t, z

def ecg_publisher_node():
    rospy.init_node('ecg_mcsharry_publisher', anonymous=True)
    pub = rospy.Publisher('ecg_signal', Float32, queue_size=10)

    fs = 60  # frequenza campionamento
    T_max = 10  # durata in secondi

    t, ecg_signal = generate_ecg_signal(T_max, fs)

    # Aggiungi rumore gaussiano al segnale
    noise_std = 0.01
    noise = np.random.normal(0, noise_std, size=ecg_signal.shape)
    noisy_ecg_signal = ecg_signal + noise

    # Plot del segnale originale e rumoroso
    plt.figure(figsize=(12, 5))
    plt.plot(t, ecg_signal, label='ECG sintetico originale')
    plt.plot(t, noisy_ecg_signal, label='ECG sintetico con rumore', alpha=0.7)
    plt.title('Segnale ECG sintetico (modello McSharry)')
    plt.xlabel('Tempo [s]')
    plt.ylabel('Ampiezza')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    rate = rospy.Rate(fs)
    rospy.loginfo("Inizio pubblicazione segnale ECG sintetico su 'ecg_signal'")

    i = 0
    while not rospy.is_shutdown() and i < len(noisy_ecg_signal):
        msg = Float32()
        msg.data = noisy_ecg_signal[i]
        pub.publish(msg)
        i += 1
        rate.sleep()

    rospy.loginfo("Fine pubblicazione segnale ECG.")

if __name__ == '__main__':
    try:
        ecg_publisher_node()
    except rospy.ROSInterruptException:
        pass
