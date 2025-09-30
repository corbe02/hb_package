#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import numpy as np
import re
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, detrend
from sklearn.decomposition import PCA
# Non abbiamo bisogno di mpl_toolkits o rfft in questo nodo di pubblicazione, 
# ma manteniamo le importazioni essenziali per l'elaborazione.

# --- Parametri Globali ---
FILENAME = "/home/corbe/heart_ws/src/heart_pkg/positions/tracked_features5.txt"
FS = 60 # Frequenza di campionamento in Hz
TOPIC_NAME = "/ecg_signal"

# ----------------------- 1. Funzioni di Supporto (Parsing ed Estrazione) -----------------------

def parse_file(fname):
    """Legge il file di coordinate e restituisce un dizionario {PointID: [(x, y, z), ...]}."""
    data = {}
    try:
        with open(fname, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        rospy.logerr(f"File non trovato: {fname}")
        return data

    blocks = re.split(r'(?=PointID:\d+)', content)
    for block in blocks:
        if not block.strip():
            continue
        # Usa re.DOTALL (re.S) per far matchare il punto (.) anche con newline
        m = re.match(r'PointID:(\d+).*?History:((?:\([^)]+\))+)', block, flags=re.S)
        if not m:
            continue
        pid = int(m.group(1))
        history_str = m.group(2)
        triplets = re.findall(r'\(([^)]+)\)', history_str)
        coords = []
        for t in triplets:
            parts = [p.strip() for p in t.split(',')]
            if len(parts) != 3:
                continue
            try:
                x, y, z = map(float, parts)
                coords.append((x, y, z))
            except ValueError:
                rospy.logwarn(f"Salto un tripletto non valido in PointID {pid}.")
                continue
        if coords:
            data[pid] = coords
    
    if not data:
        rospy.logwarn("Nessun dato di feature valido trovato.")
    return data


def extract_signal_pca(coords, fs=60, n_frames=100):
    """
    Estrae il segnale 1D dal movimento 3D usando PCA e lo filtra 
    nella banda tipica del battito (0.5 Hz - 4 Hz).
    """
    # 1. PCA per stimare direzione principale (sui primi n_frames per robustezza)
    n = min(len(coords), n_frames)
    data = np.array(coords[:n])
    pca = PCA(n_components=1)
    pca.fit(data)
    dir_vec = pca.components_[0]

    # 2. Proiezione di tutte le coordinate sulla direzione principale
    projected = np.array(coords) @ dir_vec

    # 3. Detrend + filtro passa-banda (HR: 0.5 Hz - 4 Hz)
    projected_detr = detrend(projected)
    
    low, high = 0.5, 4.0 # Frequenza cardiaca tipica (30 BPM - 240 BPM)
    nyquist_freq = fs / 2
    
    try:
        b, a = butter(3, [low/nyquist_freq, high/nyquist_freq], btype="band")
        # Applica il filtro senza ritardo di fase
        filtered = filtfilt(b, a, projected_detr) 
    except ValueError as e:
        rospy.logerr(f"[ERRORE filtro] {e}. Restituisco solo segnale detrend.")
        filtered = projected_detr

    return filtered

def rescale_signal(signal, target_min=-0.06, target_max=0.01):
    """
    Riscala un array NumPy dal suo range attuale al range target [target_min, target_max].
    """
    # 1. Calcola il Min e Max attuali del segnale
    min_original = np.min(signal)
    max_original = np.max(signal)
    
    # 2. Evita divisione per zero nel caso di segnale piatto
    if max_original - min_original == 0:
        rospy.logwarn("Il segnale è piatto; non può essere riscalato dinamicamente.")
        return np.full_like(signal, (target_min + target_max) / 2) # Restituisce il valore medio

    # 3. Applicazione della formula Min-Max Scaling
    # Normalizzazione [0, 1]
    normalized_signal = (signal - min_original) / (max_original - min_original)
    
    # Riscalatura al target [target_min, target_max]
    scaled_signal = normalized_signal * (target_max - target_min) + target_min
    
    # Debug per verifica
    rospy.loginfo(f"Riscalatura completata. Nuovo range: [{np.min(scaled_signal):.4f}, {np.max(scaled_signal):.4f}]")
    
    return scaled_signal

# ----------------------- 2. Funzione Nodo ROS -----------------------

def ecg_publisher_node(signal, fs):
    """
    Pubblica i campioni del segnale estratto sul topic ROS una sola volta.
    """
    if not signal.size:
        rospy.logerr("Impossibile pubblicare: il segnale è vuoto.")
        return

    pub = rospy.Publisher(TOPIC_NAME, Float32, queue_size=10)
    
    # Imposta il rate di pubblicazione al rate di campionamento originale (FS)
    rate = rospy.Rate(fs) 

    rospy.loginfo(f"Avvio pubblicazione singola del segnale su {TOPIC_NAME} a {fs} Hz.")
    
    for i, sample_value in enumerate(signal):
        if rospy.is_shutdown():
            break

        msg = Float32()
        msg.data = float(sample_value)
        pub.publish(msg)

        # Debug: logga ogni 1 secondo (60 campioni a 60 Hz)
        if i % fs == 0:
            rospy.logdebug(f"Pubblicato campione {i}: {sample_value:.4f}")

        rate.sleep()
    
    rospy.loginfo("Segnale pubblicato una sola volta. Nodo bloccato.")



# ----------------------- 3. MAIN (Inizializzazione e Flusso Dati) -----------------------

if __name__ == "__main__":
    rospy.init_node('ecg_signal_generator', anonymous=True)
    
    rospy.loginfo("Fase 1: Caricamento e Processamento Dati 3D.")
    
    # 1. Carica e processa i dati
    data = parse_file(FILENAME)
    
    if not data:
        rospy.logerr("Impossibile procedere: Dati non disponibili.")
        exit()

    # 2. Estrazione del segnale 
    first_pid = list(data.keys())[0]
    coords = data[first_pid]
    
    # first_proj è il segnale "ECG" filtrato
    first_proj = extract_signal_pca(coords, fs=FS, n_frames=100)
    
    rospy.loginfo(f"Segnale estratto originale. Range: [{np.min(first_proj):.4f}, {np.max(first_proj):.4f}]")

    # --- NUOVO: RISCALATURA DEL SEGNALE ---
    
    # Target: da -0.06 a 0.01
    TARGET_MIN = -0.06
    TARGET_MAX = 0.01
    
    # Riscala il segnale per l'ingresso del tuo Particle Filter
    first_proj_scaled = rescale_signal(first_proj, TARGET_MIN, TARGET_MAX)
    
    rospy.loginfo(f"Fase 2: Segnale riscalato pronto per la pubblicazione. Lunghezza {len(first_proj_scaled)} campioni.")
    # --- PLOT DEL SEGNALE PRIMA DELLA PUBBLICAZIONE ---
    plt.figure(figsize=(12, 5))
    plt.plot(np.arange(len(first_proj_scaled)) / FS, first_proj_scaled, color='orange', label='ECG (scaled)')
    plt.xlabel("Time [s]")
    plt.ylabel("Amplitude")
    plt.title("Segnale ECG filtrato e riscalato")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # 3. Pubblica il segnale riscalato su ROS
    try:
        # Nota: usiamo first_proj_scaled invece di first_proj
        ecg_publisher_node(first_proj_scaled, FS) 
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Nodo di pubblicazione terminato.")