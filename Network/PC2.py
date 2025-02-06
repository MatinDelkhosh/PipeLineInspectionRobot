import cv2
import socket
import pickle
import struct
import threading
import tkinter as tk
from tkinter import Button
from PIL import Image, ImageTk
import datetime

# Socket setup
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '0.0.0.0'  # Listen on all interfaces
port = 9999
server_socket.bind((host_ip, port))
server_socket.listen(5)
print(f"Listening on {host_ip}:{port}")

# Accept a connection
client_socket, addr = server_socket.accept()
print(f"Connection from: {addr}")

# Tkinter setup
root = tk.Tk()
root.title("Video Feed")

# Create a label for the video feed
video_label = tk.Label(root)
video_label.pack()

# Global variable to store the current frame
current_frame = None

# Function to save the current frame
def save_image():
    global current_frame
    if current_frame is not None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"captured_image_{timestamp}.jpg"
        cv2.imwrite(filename, current_frame)
        print(f"Image saved as {filename}")

# Function to send commands to the client
def send_command(command):
    try:
        client_socket.sendall(command.encode('utf-8'))
        print(f"Command sent: {command}")
    except Exception as e:
        print(f"Error sending command: {e}")

# Buttons
save_button = Button(root, text="Save Image", command=save_image)
save_button.pack()
stop_button = Button(root, text="Stop Motor", command=lambda: send_command("STOP"))
stop_button.pack()
start_button = Button(root, text="Start Motor", command=lambda: send_command("START"))
start_button.pack()

# Function to receive video frames in a separate thread
def receive_video():
    global current_frame
    data = b""
    payload_size = struct.calcsize("L")
    try:
        while True:
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    break
                data += packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]
            while len(data) < msg_size:
                packet = client_socket.recv(4096)
                if not packet:
                    break
                data += packet
            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)
            current_frame = frame
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()
        server_socket.close()

# Start video receiving in a separate thread
threading.Thread(target=receive_video, daemon=True).start()

# Start the Tkinter main loop
root.mainloop()
