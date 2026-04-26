# Data Acquisition

This project records accelerometer data on the ESP32 wearable and streams it to a computer, where it is saved as CSV files for offline analysis.

## What It Does

- Initializes the MPU6050 IMU over I2C.
- Connects the wearable to Wi-Fi in station mode.
- Waits for a hardware button press to start a recording session.
- Streams accelerometer samples at 20 Hz as newline-delimited JSON to a TCP server.
- Stops recording on the next button press, sends a final marker, and closes the socket.
- The host server writes one CSV file per session UUID.

## Implementation Overview

### Firmware (ESP-IDF)

Main logic is implemented in main/main.c and main/imu_driver.c.

- IMU driver:
  - Uses ESP-IDF I2C master API.
  - Talks to MPU6050 at address 0x68.
  - Reads raw accel registers and converts values to g units.

- Wi-Fi and networking:
  - Uses ESP-IDF Wi-Fi STA mode with reconnect handling.
  - Connects to a configured TCP endpoint (SERVER_IP, SERVER_PORT).

- Recording control:
  - Button on GPIO39 toggles session start/stop.
  - Debounce and edge detection are applied in software.

- Session handling:
  - Generates an RFC 4122 style UUID (v4) per recording session.
  - Buffers readings and sends in batches to reduce socket overhead.

- Sampling and transmission:
  - Sampling rate: 20 Hz.
  - Batch size: 10 readings before send.
  - Payload format per reading:
    {"uuid": "<session-id>", "x": <float>, "y": <float>, "z": <float>}
  - Final message when stopping:
    {"uuid": "<session-id>", "final": true}

### Host Server

The host receiver is server/server.py.

- Listens on 0.0.0.0:8080.
- Accepts multiple TCP clients (thread per connection).
- Parses newline-delimited JSON lines.
- Buffers readings by session UUID in memory.
- On final=true, writes data/session-uuid.csv with columns x,y,z.

This separates data capture on the wearable from data persistence on the computer.

## Project Structure

- main/main.c: app entrypoint, Wi-Fi, button handling, session streaming loop.
- main/imu_driver.c, main/imu_driver.h: MPU6050 I2C initialization and reads.
- main/secrets.h: local Wi-Fi credentials (not for version control).
- server/server.py: TCP receiver and CSV writer.
- server/data/: output folder for recorded CSV files.

## How To Run

### 1. Configure firmware credentials and server endpoint

- Set Wi-Fi values in main/secrets.h (or via compile-time defines).
- Set SERVER_IP in main/main.c to your computer IP on the same network.
- Ensure SERVER_PORT matches the Python server (default 8080).

### 2. Start the Python server on your computer

From data_aquisition/server:

python3 server.py

### 3. Build and flash firmware

From data_aquisition:

idf.py set-target esp32
idf.py build
idf.py flash monitor

### 4. Record

- Press the device button once to start recording.
- Press again to stop.
- The server will save a CSV file in server/data/ using the session UUID as filename.

## Notes

- A stable Wi-Fi connection is required before streaming starts.
- If connection drops, firmware retries and can be canceled by pressing the button.
- Current CSV output contains x, y, z only (no explicit timestamp column).
