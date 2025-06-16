#!/usr/bin/env python3
import rospy
import neurokit2 as nk
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
import time
import numpy as np

def ecg_node():
    rospy.init_node('ecg_simulator_plotter', anonymous=True)
    ecg_pub = rospy.Publisher('ecg_signal', Float32, queue_size=10)

    try:
        duration = 5  # secondi
        sampling_rate = 60 # Hz
        ecg = nk.ecg_simulate(duration=duration, sampling_rate=sampling_rate, method="ecgsyn")
        t = np.linspace(0, duration, len(ecg))

        mean = np.mean(ecg)
        std = np.std(ecg)
        print(mean)
        print(std)

        noise_std = 0.05 
        noise = np.random.normal(loc=0.0, scale=noise_std, size=len(ecg))
        noisy_heartbeat = ecg + noise

        # # NOISE ADDITION
        # # 1. Low-frequency drift
        # drift = 0.2 * np.sin(2 * np.pi * 0.5 * t)

        # # 2. Laplace noise
        # laplace_noise = np.random.laplace(loc=0.0, scale=0.1, size=t.shape)

        # # 3. Impulse noise
        # impulses = np.zeros_like(t)
        # impulse_indices = np.random.choice(len(t), size=10, replace=False)
        # impulses[impulse_indices] = np.random.uniform(-1, 1, size=10)

        # # 4. Multiplicative noise
        # multiplicative_envelope = 1 + 0.1 * np.random.randn(*t.shape)

        # # Final noisy signal
        # noisy_heartbeat = (ecg + drift + laplace_noise + impulses) * multiplicative_envelope

        plt.figure(figsize=(12, 5))
        plt.plot(t, ecg, label="ECG originale", alpha=0.7)
        plt.plot(t, noisy_heartbeat, label="ECG con rumore", alpha=0.7)
        plt.title("Confronto tra ECG originale e con rumore")
        plt.xlabel("Tempo (s)")
        plt.ylabel("Ampiezza")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # Pubblicazione su ROS
        rate = 60
        sleep_time = 1.0 / rate
        rospy.loginfo(f"Lunghezza del segnale ECG: {len(ecg)}, Frequenza di pubblicazione: {rate} Hz")

        for i, sample in enumerate(noisy_heartbeat):
            if rospy.is_shutdown():
                rospy.loginfo("Ricevuto segnale di shutdown, terminando la pubblicazione.")
                break
            ecg_msg = Float32()
            ecg_msg.data = sample
            ecg_pub.publish(ecg_msg)
            rospy.loginfo(f"Pubblicato campione {i}: {sample}")
            time.sleep(sleep_time)

        rospy.loginfo("Pubblicazione del segnale ECG completata.")
        
    except Exception as e:
        rospy.logerr(f"Si è verificato un errore: {e}")

if __name__ == '__main__':
    try:
        ecg_node()
    except Exception as e:
        rospy.logerr(f"Si è verificato un errore: {e}")
