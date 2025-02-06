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
frame.pack()

# Create a label for the video feed
video_label = tk.Label(frame)
video_label.pack(side=tk.LEFT)

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
                points_3d.append(data_content)
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

# Embed the Matplotlib figure into Tkinter
canvas = FigureCanvasTkAgg(fig, master=frame)  # Parent widget is button_frame
canvas.get_tk_widget().pack(side=tk.LEFT)

# Function to update the 3D plot with new points
def update_3d_plot():
    global points_3d
    RADIUS = 0.15

    if len(points_3d) >= 2:
        ax.clear()
        num_circle_points = 50  # Number of points to form a smooth circle
        theta = np.linspace(0, 2 * np.pi, num_circle_points)

        for i in range(len(points_3d) - 1):
            start = np.array(points_3d[i])
            end = np.array(points_3d[i+1])

            for j,end in enumerate(points_3d[i:]):
                distance = np.sqrt((start[0]-end[0])**2 + (start[1]-end[1])**2)
                if distance > 0.1:
                    break
            i += j
            
            # Compute tangent vector (direction of the pipe)
            tangent = end - start
            tangent /= np.linalg.norm(tangent)  # Normalize

            # Find an arbitrary normal vector that is perpendicular to the tangent
            if np.allclose(tangent, [1, 0, 0]):  
                normal = np.array([0, 1, 0])  # If tangent is along x-axis, choose y-axis as normal
            else:
                normal = np.cross(tangent, [1, 0, 0])  # Cross product with x-axis
                normal /= np.linalg.norm(normal)  # Normalize

            # Compute the binormal vector (perpendicular to both tangent and normal)
            binormal = np.cross(tangent, normal)
            binormal /= np.linalg.norm(binormal)

            # Generate points for the circular cross-section
            circle_points = np.array([
                RADIUS * np.cos(theta)[:, None] * normal + 
                RADIUS * np.sin(theta)[:, None] * binormal
            ]).squeeze()

            # Interpolate positions between start and end
            for alpha in np.linspace(0, 1, 10):  # Adjust density of circles
                position = start * (1 - alpha) + end * alpha
                x_circle = position[0] + circle_points[:, 0]
                y_circle = position[1] + circle_points[:, 1]
                z_circle = position[2] + circle_points[:, 2]
                ax.plot(x_circle, y_circle, z_circle, color="blue", alpha=0.3)

        # Plot the main pipe path
        xs, ys, zs = zip(*points_3d)
        ax.plot(xs, ys, zs, color="red", linewidth=3, label="Pipe Path")
        ax.scatter(xs, ys, zs, color="black", s=50, label="Points")

        # Label and refresh
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Pipe Profile with Defined Diameter")
        ax.legend()
        canvas.draw()
        print('change canvas')

# Start the Tkinter main loop
root.mainloop()
