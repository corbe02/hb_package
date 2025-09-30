#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np

# ------------------- 1. Caricamento dati -------------------
filename = "/tmp/particle_filter_log.csv"
df = pd.read_csv(filename)

# Assicurati che siano numerici
df["time"] = pd.to_numeric(df["time"], errors="coerce")
df["z"] = pd.to_numeric(df["z"], errors="coerce")
df["est"] = pd.to_numeric(df["est"], errors="coerce")

# Controllo rapido
print(df.dtypes)
print(df.head())

# ------------------- 2. Parametri -------------------
min_height = df["est"].max() * 0.2  # soglia per picchi più alti
fs = 1 / np.median(np.diff(df["time"]))  # frequenza di campionamento approssimativa
min_distance_samples = int(fs * 0.3)  # distanza minima tra picchi 0.3s

# ------------------- 3. Trova tutti i picchi -------------------
peaks, _ = find_peaks(df["est"], distance=min_distance_samples, height=min_height)

# ------------------- 4. Finestra centrale 30s -------------------
window_sec = 30
mid_time = (df["time"].iloc[0] + df["time"].iloc[-1]) / 2
t0 = mid_time - window_sec / 2
t1 = mid_time + window_sec / 2

# Picchi nella finestra centrale
peaks_central = peaks[(df["time"].iloc[peaks] >= t0) & (df["time"].iloc[peaks] < t1)]

# Frequenza cardiaca nella finestra centrale
hr_central = len(peaks_central) * (60 / window_sec)

print(f"Intervallo centrale {t0:.1f}-{t1:.1f}s:")
print(f"Numero di picchi rilevati: {len(peaks_central)}")
print(f"Frequenza cardiaca stimata: HR ~ {hr_central:.1f} bpm")

# ------------------- 5. Plot -------------------
plt.figure(figsize=(12, 5))
plt.plot(df["time"], df["z"], label="Z (raw)")
plt.plot(df["time"], df["est"], label="Estimate", color="orange")

# Picchi centrali evidenziati
plt.plot(df["time"].iloc[peaks_central], df["est"].iloc[peaks_central], "rx", label="High Peaks (central)")

# Evidenzia la finestra centrale sul plot
plt.axvspan(t0, t1, color='yellow', alpha=0.2, label="30s Central Window")

plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.title("ECG Signal - Picchi nella finestra centrale")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
