#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ------------------- 1. Caricamento dati -------------------
filename = "/tmp/particle_filter_mech.csv"
df = pd.read_csv(filename)

# Conversione a numerico
df["time"] = pd.to_numeric(df["time"], errors="coerce")
df["measurement"] = pd.to_numeric(df["measurement"], errors="coerce")
df["estimate"] = pd.to_numeric(df["estimate"], errors="coerce")
df = df.dropna(subset=["time", "measurement", "estimate"]).reset_index(drop=True)

# ------------------- 2. Ricostruzione tempo coerente -------------------
FS = 60.0  # frequenza reale di campionamento

# Ricrea tempo coerente da zero
df["time_fixed"] = np.arange(len(df)) / FS

# ------------------- 3. Plot: Originale vs Filtrato -------------------
plt.figure(figsize=(14, 6))

# Traccia il segnale originale e quello stimato
plt.plot(df["time_fixed"], df["measurement"], label="Z (raw)", alpha=0.6)
plt.plot(df["time_fixed"], df["estimate"], label="Estimate (Particle Filter)", color="orange", linewidth=1.2)

# Aggiungi etichette e stile
plt.xlabel("Tempo [s]")
plt.ylabel("Ampiezza")
plt.title("Confronto: Segnale Originale vs Filtrato (Particle Filter)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.5)
plt.tight_layout()
plt.show()
