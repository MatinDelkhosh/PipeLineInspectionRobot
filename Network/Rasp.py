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
        # Receive command from the PC
        command_data = client_socket.recv(1024).decode('utf-8')
        if command_data:
            # Process the command (e.g., start/stop and speed)
            print(f"Received command: {command_data}")

        # Capture frame from the camera
        frame = picam2.capture_array()

        # Process the frame (e.g., convert to grayscale)
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Serialize the frame
        data = pickle.dumps(processed_frame)
        message_size = struct.pack("L", len(data))  # Pack the length of the data

        # Send the frame and 3D points over the network
        # Assuming 3D points are generated or available
        points_3d = [(0, 0, 0), (1, 1, 1)]  # Example 3D points
        points_data = pickle.dumps(points_3d)
        client_socket.sendall(message_size + data + points_data)

except KeyboardInterrupt:
    print("Streaming stopped")

finally:
    client_socket.close()
    picam2.stop()
