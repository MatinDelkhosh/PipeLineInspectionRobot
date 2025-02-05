import cv2
import socket
import pickle
import struct
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# Socket setup
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '192.168.171.250'  # Replace with your PC's IP address
port = 9999
client_socket.connect((host_ip, port))

try:
    while True:
        # Capture frame from the camera
        frame = picam2.capture_array()

        # Process the frame (e.g., convert to grayscale)
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Serialize the frame
        data = pickle.dumps(frame)
        message_size = struct.pack("L", len(data))  # Pack the length of the data

        # Send the frame over the network
        client_socket.sendall(message_size + data)

except KeyboardInterrupt:
    print("Streaming stopped")

finally:
    client_socket.close()
    picam2.stop()