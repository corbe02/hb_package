import os
import pandas as pd
import matplotlib.pyplot as plt

log_path = "/tmp/particle_filter_log.csv"

# Verifica se il file esiste e non è vuoto
if os.path.isfile(log_path) and os.path.getsize(log_path) > 0:
    df = pd.read_csv(log_path, header=None, names=["time", "measurement", "estimate"])

    if not df.empty:
        plt.plot(df["time"], df["measurement"], label="Measurement")
        plt.plot(df["time"], df["estimate"], label="Filtered (Estimate)")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Signal")
        plt.title("Measurement vs Filtered Signal")
        plt.show()
    else:
        print("⚠️ Il file esiste ma non contiene dati.")
else:
    print("Il file di log non esiste o è vuoto.")
