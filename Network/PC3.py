import cv2
import socket
import struct
import pickle

# Setup socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(("192.168.171.150", 9999))  # Replace with your Raspberry Pi's IP

data = b""
payload_size = struct.calcsize(">L")

try:
    while True:
        # Receive frame size
        while len(data) < payload_size:
            packet = client_socket.recv(4096)
            if not packet:
                break
            data += packet
        packed_size = data[:payload_size]
        data = data[payload_size:]
        frame_size = struct.unpack(">L", packed_size)[0]

        # Receive frame data
        while len(data) < frame_size:
            packet = client_socket.recv(4096)
            if not packet:
                break
            data += packet
        
        frame_data = data[:frame_size]
        data = data[frame_size:]

        # Deserialize frame
        frame = pickle.loads(frame_data)

        # Display frame
        cv2.imshow("Video Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("Closing connection...")
finally:
    client_socket.close()
    cv2.destroyAllWindows()
