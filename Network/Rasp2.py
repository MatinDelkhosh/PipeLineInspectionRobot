import cv2
import socket
import struct
import pickle
from picamera2 import Picamera2

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

# Setup socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 8485))  # Listen on all interfaces, port 8485
server_socket.listen(1)
print("Waiting for connection...")

conn, addr = server_socket.accept()
print(f"Connection from {addr}")

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # Convert to grayscale

        # Serialize frame
        data = pickle.dumps(frame)
        conn.sendall(struct.pack(">L", len(data)) + data)  # Send size + data

except KeyboardInterrupt:
    print("Stopping...")
finally:
    conn.close()
    server_socket.close()
    picam2.close()
