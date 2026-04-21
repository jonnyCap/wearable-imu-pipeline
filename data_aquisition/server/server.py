import socket
import json
import csv
import logging
import threading
from collections import defaultdict
import os

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

HOST = '0.0.0.0'
PORT = 8080
OUTPUT_DIR = 'data'

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Buffer for storing session data: UUID -> list of {x, y, z}
session_data = defaultdict(list)
# Lock for thread-safe access to session_data
data_lock = threading.Lock()

def handle_client(conn, addr):
    """Handles incoming data from a single client."""
    logging.info(f"Connected by {addr}")
    with conn:
        buffer = ""
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    break
                
                buffer += data.decode('utf-8')
                
                # Assume delimited JSON messages (e.g., newline separated)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    
                    try:
                        payload = json.loads(line)
                        session_uuid = payload.get('uuid')
                        is_final = payload.get('final', False)
                        
                        if not session_uuid:
                            logging.warning("Received payload without UUID")
                            continue
                            
                        if is_final:
                            logging.info(f"Received final flag for session {session_uuid}. Saving to CSV.")
                            save_to_csv(session_uuid)
                        else:
                            # Extract acceleration data
                            x = float(payload.get('x', 0.0))
                            y = float(payload.get('y', 0.0))
                            z = float(payload.get('z', 0.0))
                            
                            with data_lock:
                                session_data[session_uuid].append({'x': x, 'y': y, 'z': z})
                            
                    except json.JSONDecodeError:
                        logging.error(f"Failed to parse JSON: {line}")
                    except ValueError:
                         logging.error(f"Failed to parse payload values as floats: {line}")
            except ConnectionResetError:
                logging.warning(f"Connection reset by {addr}")
                break

def save_to_csv(session_uuid):
    """Saves buffered data for a UUID to a CSV file and clears the buffer."""
    with data_lock:
        data = session_data.pop(session_uuid, None)
        
    if not data:
        logging.warning(f"No data found for session {session_uuid} to save.")
        return
        
    filename = os.path.join(OUTPUT_DIR, f"{session_uuid}.csv")
    try:
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['x', 'y', 'z']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for row in data:
                writer.writerow(row)
                
        logging.info(f"Successfully saved {len(data)} records to {filename}")
    except Exception as e:
        logging.error(f"Failed to save CSV for session {session_uuid}: {e}")

def main():
    """Starts the TCP server."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        logging.info(f"TCP Data Acquisition Server listening on {HOST}:{PORT}")
        logging.info(f"Saving recorded CSV files to '{OUTPUT_DIR}' directory.")
        
        try:
            while True:
                conn, addr = server_socket.accept()
                # Create a new thread for each client connection if needed,
                # but for simple sequential embedded device sending, this works.
                client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
        except KeyboardInterrupt:
             logging.info("Server manually stopped.")

if __name__ == "__main__":
    main()
