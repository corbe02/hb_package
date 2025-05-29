#!/usr/bin/env python3
import rospy
import neurokit2 as nk
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
import time

def ecg_node():
    rospy.init_node('ecg_simulator_plotter', anonymous=True)
    ecg_pub = rospy.Publisher('ecg_signal', Float32, queue_size=10)

    try:
        ecg = nk.ecg_simulate(duration=1000, method="ecgsyn")
        # Frequenza di pubblicazione (in Hz)
        rate = 60
        rospy.loginfo(f"Lunghezza del segnale ECG: {len(ecg)}, Frequenza di pubblicazione: {rate} Hz")
        sleep_time = 1.0 / rate

        # Pubblica ogni punto del segnale ECG
        for i, sample in enumerate(ecg):
            if rospy.is_shutdown():  # Controlla se ROS è in fase di shutdown
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