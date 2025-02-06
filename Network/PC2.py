import cv2
import socket
import pickle
import struct
import threading
import tkinter as tk
from tkinter import Button
from PIL import Image, ImageTk
import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Socket setup
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '0.0.0.0'  # Listen on all interfaces
port = 9999
server_socket.bind((host_ip, port))
server_socket.listen(5)
print(f"Listening on {host_ip}:{port}")

# Accept a connection
client_socket, addr = None, None
def accept_connection():
    global client_socket, addr
    client_socket, addr = server_socket.accept()
    print(f"Connection from: {addr}")

threading.Thread(target=accept_connection, daemon=True).start()

# Tkinter setup
root = tk.Tk()
root.title("Video Feed")

# Create a frame for the layout
frame = tk.Frame(root)
frame.pack(side=tk.LEFT)

# Create a label for the video feed
video_label = tk.Label(frame)
video_label.pack()

# Global variable to store the current frame
current_frame = None
points_3d = []

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
# Create a frame for the buttons
button_frame = tk.Frame(root)
button_frame.pack()

# Buttons for control
save_button = Button(button_frame, text="Save Image", command=lambda: save_image())
save_button.pack()

start_button = Button(button_frame, text="Start Motor", command=lambda: send_command("START"))
start_button.pack(side=tk.LEFT, padx=5)

stop_button = Button(button_frame, text="Stop Motor", command=lambda: send_command("STOP"))
stop_button.pack(side=tk.LEFT, padx=5)

# Function to receive video frames in a separate thread
def receive_data():
    global current_frame, client_socket, points_3d
    while client_socket is None: pass
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

            # Extract the frame data and the data type identifier
            frame_data = data[:msg_size]
            data = data[msg_size:]
            data_type, data_content = pickle.loads(frame_data)  # Unpack the data

            if data_type == 'image':
                frame = data_content
                current_frame = frame
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                imgtk = ImageTk.PhotoImage(image=img)
                video_label.imgtk = imgtk
                video_label.configure(image=imgtk)

            elif data_type == 'points_3d':
                points_3d = data_content
                print('recieved points')
                update_3d_plot()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()
        server_socket.close()

# Start video receiving in a separate thread
threading.Thread(target=receive_data, daemon=True).start()

# 3D plot setup
fig = plt.Figure(figsize=(6, 4), dpi=100)
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Function to update the 3D plot with new points
def update_3d_plot():
    global points_3d
    ax.clear()  # Clear previous plot
    if points_3d:
        points_3d_array = np.array(points_3d)
        ax.scatter(points_3d_array[:, 0], points_3d_array[:, 1], points_3d_array[:, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    canvas.draw()

# Embed the Matplotlib figure into Tkinter
canvas = FigureCanvasTkAgg(fig, master=frame)  # Parent widget is button_frame
canvas.get_tk_widget().pack(side=tk.LEFT)

# Start the Tkinter main loop
root.mainloop()
