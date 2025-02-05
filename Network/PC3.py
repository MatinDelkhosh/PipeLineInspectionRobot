import cv2
import socket
import pickle
import struct

# Socket setup
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '192.168.171.250'  # Replace with your PC's IP address
port = 9999
server_socket.bind((host_ip, port))
server_socket.listen(5)
print(f"Listening on {host_ip}:{port}")

# Accept a connection
client_socket, addr = server_socket.accept()
print(f"Connection from: {addr}")

data = b""
payload_size = struct.calcsize("L")

try:
    while True:
        # Retrieve message size
        while len(data) < payload_size:
            data += client_socket.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]

        # Retrieve all data based on message size
        while len(data) < msg_size:
            data += client_socket.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        # Deserialize the frame
        frame = pickle.loads(frame_data)

        # Display the frame
        cv2.imshow("Received Frame", frame)
        if cv2.waitKey(1) == 27:  # Press 'Esc' to exit
            break

except KeyboardInterrupt:
    print("Streaming stopped")

finally:
    client_socket.close()
    cv2.destroyAllWindows()