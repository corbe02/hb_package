#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np

# ------------------- 1. Caricamento dati -------------------
filename = "/tmp/particle_filter_log.csv"
df = pd.read_csv(filename)

# Conversione a numerico
df["time"] = pd.to_numeric(df["time"], errors="coerce")
df["z"] = pd.to_numeric(df["z"], errors="coerce")
df["est"] = pd.to_numeric(df["est"], errors="coerce")
df = df.dropna(subset=["time", "z", "est"]).reset_index(drop=True)

# ------------------- 2. Ricostruzione tempo coerente -------------------
FS = 20.0  # frequenza reale di campionamento

# Se i timestamp non sono affidabili (es. UNIX time), li ricreiamo da zero:
duration_s = len(df) / FS
df["time_fixed"] = np.arange(len(df)) / FS

print(f"Durata segnale ricostruita: {duration_s:.1f} s ({len(df)} campioni a {FS} Hz)")

# ------------------- 3. Parametri picchi -------------------
min_height = df["est"].std() * 0.005  # soglia relativa

   # soglia (20% del massimo)
min_distance_samples = int(FS * 0.5)      # distanza minima (0.3 s → 200 bpm max)

# ------------------- 4. Trova picchi -------------------
peaks, properties = find_peaks(df["est"], distance=min_distance_samples, prominence=0.01)

# ------------------- 5. Finestra centrale (30 s) -------------------
window_sec = 30
mid_time = df["time_fixed"].iloc[-1] / 2
t0, t1 = mid_time - window_sec / 2, mid_time + window_sec / 2

mask_central = (df["time_fixed"].iloc[peaks] >= t0) & (df["time_fixed"].iloc[peaks] < t1)
peaks_central = peaks[mask_central]

hr_central = len(peaks_central) * (60 / window_sec)

print(f"\n⏱ Intervallo centrale {t0:.1f}-{t1:.1f} s:")
print(f"   Picchi rilevati: {len(peaks_central)}")
print(f"   Frequenza cardiaca stimata: {hr_central:.1f} bpm\n")

# ------------------- 6. Plot -------------------
plt.figure(figsize=(14, 6))

plt.plot(df["time_fixed"], df["z"], label="Z (raw)", alpha=0.6)
plt.plot(df["time_fixed"], df["est"], label="Estimate (Particle Filter)", color="orange", linewidth=1.2)

plt.plot(df["time_fixed"].iloc[peaks], df["est"].iloc[peaks], "kx", label="Tutti i picchi")
plt.plot(df["time_fixed"].iloc[peaks_central], df["est"].iloc[peaks_central], "ro", markersize=6, label="Picchi (30s centrali)")

plt.axvspan(t0, t1, color='yellow', alpha=0.2, label="Finestra centrale (30 s)")

plt.text(t1 + 1, np.max(df["est"]) * 0.8,
         f"FS = {FS:.1f} Hz\nHR ≈ {hr_central:.1f} bpm",
         fontsize=10, bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray'))

plt.xlabel("Tempo [s]")
plt.ylabel("Ampiezza")
plt.title("Segnale ECG stimato dal Particle Filter")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.5)
plt.tight_layout()
plt.show()
