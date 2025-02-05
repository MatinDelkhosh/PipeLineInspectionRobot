import cv2
import socket
import pickle
import struct
import numpy as np
import matplotlib.pyplot as plt

# Set up the socket for receiving the data
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '0.0.0.0'  # Listen on all interfaces
port = 9999
server_socket.bind((host_ip, port))
server_socket.listen(5)
print("Waiting for connection...")
client_socket, addr = server_socket.accept()
print(f"Connection from: {addr}")

# Create a figure for plotting 3D points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Display window for the live camera feed
cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

while True:
    try:
        # Receive the message size
        message_size_data = client_socket.recv(4)
        if len(message_size_data) == 0:
            break
        message_size = struct.unpack("L", message_size_data)[0]

        # Receive the image frame
        data = b""
        while len(data) < message_size:
            data += client_socket.recv(4096)

        frame = pickle.loads(data)

        # Receive the 3D points data
        points_size_data = client_socket.recv(4)
        points_size = struct.unpack("L", points_size_data)[0]
        
        points_data = b""
        while len(points_data) < points_size:
            points_data += client_socket.recv(4096)

        points_3d = pickle.loads(points_data)

        # Display the frame
        cv2.imshow("Camera Feed", frame)

        # Clear the previous plot and plot the 3D points
        ax.clear()
        points_3d = np.array(points_3d)
        ax.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2])

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        # Refresh the plot
        plt.pause(0.1)

        # Check for the 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except KeyboardInterrupt:
        print("Streaming stopped.")
        break
    except Exception as e:
        print(f"Error: {e}")
        break

# Cleanup
cv2.destroyAllWindows()
client_socket.close()
server_socket.close()
