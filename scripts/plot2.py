#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ------------------- 1. Caricamento dati -------------------
filename = "/tmp/particle_filter_mech.csv"
df = pd.read_csv(filename)

# Trova la colonna corretta della stima
if "estimate_x" in df.columns:
    est_col = "estimate_x"
elif "estimate" in df.columns:
    est_col = "estimate"
else:
    raise ValueError("Nessuna colonna di stima trovata nel CSV!")

# Conversione a numerico
df["measurement"] = pd.to_numeric(df["measurement"], errors="coerce")
df[est_col] = pd.to_numeric(df[est_col], errors="coerce")
df = df.dropna(subset=["measurement", est_col]).reset_index(drop=True)

# ------------------- 2. Ricostruzione tempo coerente -------------------
FS = 60.0  # frequenza reale di campionamento
df["time_fixed"] = np.arange(len(df)) / FS

# ------------------- 3. Plot: Originale vs Filtrato -------------------
plt.figure(figsize=(14, 6))

plt.plot(df["time_fixed"], df["measurement"], label="Z (raw)", alpha=0.6)
plt.plot(df["time_fixed"], df[est_col], label="Estimate (Particle Filter)", color="orange", linewidth=1.2)

plt.xlabel("Tempo [s]")
plt.ylabel("Ampiezza")
plt.title("Confronto: Segnale Originale vs Filtrato (Particle Filter)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.5)
plt.tight_layout()
plt.show()
