"""
Wearable IMU Data Acquisition Server

A lightweight TCP server that receives streamed accelerometer sensor data
from the ESP32 wearable device over Wi-Fi (newline-delimited JSON format)
and persists recording sessions into individual CSV files keyed by session UUID.
"""

import argparse
import csv
import json
import logging
import os
import socket
import threading
from collections import defaultdict

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

# Buffer for storing session data: UUID -> list of {x, y, z}
session_data = defaultdict(list)
data_lock = threading.Lock()


def handle_client(conn: socket.socket, addr: tuple, output_dir: str) -> None:
    """Handles incoming data stream from a connected ESP32 client."""
    logging.info(f"Connected by {addr}")
    with conn:
        buffer = ""
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    break

                buffer += data.decode("utf-8")

                # Parse newline-delimited JSON packets
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        payload = json.loads(line)
                        session_uuid = payload.get("uuid")
                        is_final = payload.get("final", False)

                        if not session_uuid:
                            logging.warning("Received payload without UUID")
                            continue

                        if is_final:
                            logging.info(
                                f"Received final flag for session {session_uuid}. Writing CSV."
                            )
                            save_to_csv(session_uuid, output_dir)
                        else:
                            x = float(payload.get("x", 0.0))
                            y = float(payload.get("y", 0.0))
                            z = float(payload.get("z", 0.0))

                            with data_lock:
                                session_data[session_uuid].append(
                                    {"x": x, "y": y, "z": z}
                                )

                    except json.JSONDecodeError:
                        logging.error(f"Failed to parse JSON: {line}")
                    except ValueError:
                        logging.error(
                            f"Failed to parse payload values as floats: {line}"
                        )
            except ConnectionResetError:
                logging.warning(f"Connection reset by {addr}")
                break


def save_to_csv(session_uuid: str, output_dir: str) -> None:
    """Saves buffered samples for a session UUID to CSV and frees memory."""
    with data_lock:
        data = session_data.pop(session_uuid, None)

    if not data:
        logging.warning(f"No data found for session {session_uuid} to save.")
        return

    os.makedirs(output_dir, exist_ok=True)
    filename = os.path.join(output_dir, f"{session_uuid}.csv")
    try:
        with open(filename, "w", newline="") as csvfile:
            fieldnames = ["x", "y", "z"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for row in data:
                writer.writerow(row)

        logging.info(f"Successfully saved {len(data)} records to {filename}")
    except Exception as e:
        logging.error(f"Failed to save CSV for session {session_uuid}: {e}")


def main() -> None:
    """Parses CLI arguments and starts the TCP data acquisition server."""
    parser = argparse.ArgumentParser(
        description="TCP Data Acquisition Server for ESP32 Wearable IMU"
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host address to bind to (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="TCP port to listen on (default: 8080)",
    )
    parser.add_argument(
        "--output-dir",
        default="data",
        help="Directory to save recorded CSV session files (default: data)",
    )
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((args.host, args.port))
        server_socket.listen()
        logging.info(
            f"TCP Data Acquisition Server listening on {args.host}:{args.port}"
        )
        logging.info(f"Saving recorded CSV files to '{args.output_dir}' directory.")

        try:
            while True:
                conn, addr = server_socket.accept()
                client_thread = threading.Thread(
                    target=handle_client,
                    args=(conn, addr, args.output_dir),
                    daemon=True,
                )
                client_thread.start()
        except KeyboardInterrupt:
            logging.info("Server manually stopped.")


if __name__ == "__main__":
    main()
