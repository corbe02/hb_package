#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import numpy as np
import re
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, detrend
from sklearn.decomposition import PCA

# --- PARAMETRI GLOBALI ---
FILENAME = "/home/altair/anna_ws/src/Heart_3d/positions/tracked_features2.txt"
FS = 60  # Frequenza di campionamento (Hz)
DURATION_S = 40  # Durata in secondi
TOPIC_NAME = "/ecg_signal"


# ----------------------- 1. Parsing del file di coordinate -----------------------

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
                rospy.logwarn(f"Tripletto non valido in PointID {pid}, salto.")
                continue
        if coords:
            data[pid] = coords

    if not data:
        rospy.logwarn("Nessun dato di feature valido trovato.")
    return data


# ----------------------- 2. Estrazione segnale PCA + filtro -----------------------

def extract_signal_pca_fixed(coords, fs=60, duration_s=40):
    """
    Estrae un segnale ECG-like da coordinate 3D usando PCA.
    Restituisce un segnale filtrato e della lunghezza esatta: fs * duration_s.
    """
    n_samples = int(fs * duration_s)
    coords_array = np.array(coords)

    # Se abbiamo meno campioni, interpola
    if len(coords_array) < n_samples:
        x = np.arange(len(coords_array))
        x_new = np.linspace(0, len(coords_array) - 1, n_samples)
        coords_resampled = np.stack([
            np.interp(x_new, x, coords_array[:, dim]) for dim in range(3)
        ], axis=-1)
    else:
        coords_resampled = coords_array[:n_samples]

    # PCA per estrarre la direzione principale di movimento
    pca = PCA(n_components=1)
    pca.fit(coords_resampled)
    dir_vec = pca.components_[0]
    projected = coords_resampled @ dir_vec

    # Detrend costante
    projected_detr = detrend(projected, type='constant')

    # Bandpass filtraggio intorno alla frequenza cardiaca (0.7–1.5 Hz)
    low, high = 0.7, 1.5
    nyquist_freq = fs / 2
    b, a = butter(2, [low / nyquist_freq, high / nyquist_freq], btype="band")
    filtered = filtfilt(b, a, projected_detr)
    #filtered = projected_detr 

    return filtered


# ----------------------- 3. Riscalatura -----------------------

def rescale_signal(signal, target_min=-0.06, target_max=0.01):
    """
    Riscala un array NumPy dal suo range attuale al range target [target_min, target_max].
    """
    min_original = np.min(signal)
    print("min e max")
    print(min_original)

    max_original = np.max(signal)
    print(max_original)

    if max_original - min_original == 0:
        rospy.logwarn("Segnale piatto; restituito valore medio costante.")
        return np.full_like(signal, (target_min + target_max) / 2)

    normalized_signal = (signal - min_original) / (max_original - min_original)
    scaled_signal = normalized_signal * (target_max - target_min) + target_min

    rospy.loginfo(f"Segnale riscalato: nuovo range [{np.min(scaled_signal):.4f}, {np.max(scaled_signal):.4f}]")
    return scaled_signal


# ----------------------- 4. Pubblicazione su ROS -----------------------

def ecg_publisher_node(signal, fs):
    """
    Pubblica il segnale campione per campione su un topic ROS a frequenza fs.
    """
    if not signal.size:
        rospy.logerr("Impossibile pubblicare: segnale vuoto.")
        return

    pub = rospy.Publisher(TOPIC_NAME, Float32, queue_size=10)
    rate = rospy.Rate(fs)
    rospy.loginfo(f"Avvio pubblicazione segnale su {TOPIC_NAME} ({fs} Hz)")

    for i, sample_value in enumerate(signal):
        if rospy.is_shutdown():
            break
        msg = Float32()
        msg.data = float(sample_value)
        pub.publish(msg)

        if i % fs == 0:
            rospy.logdebug(f"Pubblicato campione {i}: {sample_value:.4f}")

        rate.sleep()

    rospy.loginfo("Pubblicazione completata.")


# ----------------------- 5. MAIN -----------------------

if __name__ == "__main__":
    rospy.init_node('ecg_signal_generator', anonymous=True)

    rospy.loginfo("==> Avvio generazione segnale ECG simulato")

    # 1. Carica le coordinate dal file
    data = parse_file(FILENAME)
    if not data:
        rospy.logerr("File vuoto o formato errato. Uscita.")
        exit(1)

    # 2. Seleziona la prima feature valida
    first_pid = list(data.keys())[0]
    coords = data[first_pid]
    #rospy.loginfo(f"Feature ID {first_pid} con {len(coords)} campioni trovata.")

    # 3. Estrai il segnale filtrato
    first_proj = extract_signal_pca_fixed(coords, fs=FS, duration_s=DURATION_S)
    #rospy.loginfo(f"Segnale estratto: {len(first_proj)} campioni.")

    # 4. Riscala per il particle filter
    TARGET_MIN, TARGET_MAX = -0.04, 0.04
    first_proj_scaled = rescale_signal(first_proj, TARGET_MIN, TARGET_MAX)
    #first_proj_scaled = first_proj

    print(len(first_proj_scaled))

    # 5. Plot per verifica
    t = np.arange(len(first_proj_scaled)) / FS
    plt.figure(figsize=(12, 5))
    plt.plot(t, first_proj_scaled, color='orange', label='ECG filtrato e riscalato')
    plt.xlabel("Tempo [s]")
    plt.ylabel("Ampiezza")
    plt.title("Segnale ECG pronto per il Particle Filter")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # 6. Pubblica il segnale
    try:
        ecg_publisher_node(first_proj_scaled, FS)
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Nodo terminato.")
