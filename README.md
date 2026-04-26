# Wearable IMU Pipeline

This repository contains an end-to-end pipeline for a wearable IMU assignment: collecting accelerometer data, developing and running real-time glance + step detection on-device, and validating filter settings offline.

## Projects Overview

### 1) data_aquisition

Embedded ESP-IDF project for recording raw accelerometer data from the wearable and streaming it to a host machine over Wi-Fi/TCP.

- Runs on the ESP32 wearable and samples IMU acceleration (x, y, z).
- Sends JSON messages to the host with a session UUID.
- Supports a session finalization message so recordings can be cleanly closed and saved.

#### server/server.py

The `data_aquisition/server/server.py` file is the host-side TCP receiver for streamed sensor data.

- Listens on `0.0.0.0:8080`.
- Accepts newline-delimited JSON packets from the wearable.
- Buffers samples per `uuid` in memory.
- On `"final": true`, writes the buffered session to `data/<uuid>.csv`.

This script is the bridge between live ESP32 streaming and the CSV files used for offline analysis.

### 2) glance_step_detector

Embedded ESP-IDF project implementing on-device real-time activity features.

- Glance detection using short accelerometer windows (250 ms) and threshold-based posture classification with hysteresis.
- Step detection/counting from acceleration magnitude using a biquad filter cascade (high-pass + low-pass), thresholds, and a refractory interval.
- Display logic that shows the step count only while a glance gesture is active.

## Offline Analysis

### filtering

The `filtering/` folder contains the offline signal-analysis workflow:

- `filtering.ipynb`: notebook for exploratory analysis and filter tuning (e.g., spectrum/periodogram-based reasoning and parameter testing).
- `WalkStand.csv`: recorded sample dataset used in notebook experiments.

This part is where you validate processing choices before deploying them to the firmware projects.
