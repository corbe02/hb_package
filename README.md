# 🫀 ECG Signal Filtering with Particle Filter (ROS)

This ROS project simulates the generation of a noisy ECG signal and applies a **particle filter** to clean it. It is composed of two main nodes that communicate via ROS topics.

## 📁 Project Structure

```
hb_package/
├── launch/
│   ├── signal.launch              # Launches the node that generates the noisy ECG signal
│   └── ParticleFilter.launch      # Launches the particle filter node
├── scripts/
│   ├── ecg_plotter_node.py        # ECG signal generation (NeuroKit or McSharry)
│   ├── plot.py                    # For the final measured/filtered signal plot
    ├── ecg.py                     # First implementation of McSharry paper
├── include/
    ├── particle_filter.hpp       
    ├── particle.hpp              # Particle structure
    ├── utils.hpp                 # Auxilliary functions for particle filter

├── src/
│   └── particle_filter.cpp    # Applies the particle filter and publishes the filtered signal
    └── main.cpp
    └── utils.cpp              # Auxilliary functions for particle filter 
└── README.md
```

## ⚙️ Requirements

- [neurokit2](https://neurokit2.readthedocs.io/) (optional, only required for NeuroKit mode)

You can install `neurokit2` with:

```bash
pip install neurokit2
```

## 🚀 How to Run

1. **Start the noisy ECG signal publisher:**

```bash
roslaunch hb_package signal.launch
```
It shows you original and noisy signal, then once you close the plot it starts publishing.

2. **Start the particle filter node:**

```bash
roslaunch hb_package Particle_Filter.launch
```

## 🧠 ECG Signal Generation

In the file `ecg_plotter_node.py`, you can choose between two signal generation methods:

- **NeuroKit**: Uses the `neurokit2` library to generate realistic ECG signals.
- **McSharry**: A custom implementation based on the model described in the paper:

> Clifford,McSharry (2003). *A Dynamical model for Generating Synthetic Electrocardiogram Signals*.





