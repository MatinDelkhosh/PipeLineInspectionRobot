import cv2
import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import simpledialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import struct
import pickle
import io
from PIL import Image, ImageTk

# Server configuration
host = '192.168.171.250'  # ***REPLACE*** with your PC's IP address
port = 9999

# Tkinter setup
root = tk.Tk()
root.title("Camera Feed and 3D Plot")
root.geometry('1200x800')

# Matplotlib setup for subplots
fig = plt.figure(figsize=(12, 6))
gs = fig.add_gridspec(1, 2)  # 1 row, 2 columns
ax_camera = fig.add_subplot(gs[0, 0])  # Camera feed subplot
ax_plot = fig.add_subplot(gs[0, 1], projection='3d')  # 3D plot subplot

ax_camera.axis('off')  # Hide axes for camera feed

# Tkinter canvas for embedding Matplotlib figure
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Camera feed label
camera_label = tk.Label(root)
camera_label.pack(pady=10)

# 3D points data
points = []

# Motor control variables
motor_state = tk.StringVar(value="Stopped")
base_speed = tk.IntVar(value=0)

# Speed control slider
speed_scale = tk.Scale(root, from_=0, to=100, orient=tk.HORIZONTAL, label="Speed", variable=base_speed)
speed_scale.pack(pady=5)

# Function to update the 3D plot
def update_plot(new_point):
    global points

    if new_point is not None:
        points.append(new_point)
        ax_plot.clear()  # Clear the previous plot
        ax_plot.set_xlabel('X')
        ax_plot.set_ylabel('Y')
        ax_plot.set_zlabel('Z')
        ax_plot.set_title("3D Plot")
        if points:
            x, y, z = zip(*points)
            ax_plot.scatter(x, y, z)  # Plot the points
        canvas.draw()  # Redraw the canvas

def update_camera_feed(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame_rgb)
    imgtk = ImageTk.PhotoImage(image=img)
    camera_label.config(image=imgtk)
    camera_label.image = imgtk

# Function to handle button click
def on_button_click():
    global motor_state, base_speed, conn # Make sure conn is global

    current_state = motor_state.get()
    new_state = "Running" if current_state == "Stopped" else "Stopped"
    motor_state.set(new_state)
    # Get speed from slider
    speed = base_speed.get()

    # Send command to Raspberry Pi using the connected socket
    command = f"speed={speed}" if new_state == "Running" else "stop"
    try:
        conn.send(command.encode()) # Use conn.send, not sock.sendto
        print(f"Sent command: {command}")
    except Exception as e:
        print(f"Error sending command: {e}")

# Socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('0.0.0.0', port))
sock.listen(1)  # Listen for one incoming connection
print(f"Listening on {host}:{port}")

conn, addr = sock.accept()
print(f"Connected by {addr}")
conn.settimeout(2.0)

def receive_data():
    """Receives data from the client (RPi) and updates the GUI."""
    dataBuffer = b""
    payloadSize = struct.calcsize("L")

    while True:
        try:
            while len(dataBuffer) < payloadSize:
                dataBuffer += conn.recv(4096)

            packedFrameSize = dataBuffer[:payloadSize]
            dataBuffer = dataBuffer[payloadSize:]
            frameSize = struct.unpack("L", packedFrameSize)[0]

            while len(dataBuffer) < frameSize:
                dataBuffer += conn.recv(4096)

            frameData = dataBuffer[:frameSize]
            dataBuffer = dataBuffer[frameSize:]

            frame = pickle.loads(frameData)
            print('frame loaded?')
            update_camera_feed(frame)  # Assuming update_camera_feed is defined

            try:
                points_data = conn.recv(4096)
                if points_data.startswith(b'POINTS:'):
                    point = pickle.loads(points_data[7:])
                    update_plot(point)  # Assuming update_plot is defined
            except socket.timeout:
                pass  # Handle timeout for points data
            except Exception as e:
                print(f"Error receiving or processing points data: {e}")

        except socket.timeout:
            pass # Handle timeout
        except Exception as e:
            print(f"Error receiving data: {e}")
            break

# Tkinter button
button = tk.Button(root, text="Start/Stop Motor", command=on_button_click)
button.pack(pady=10)

# Start receive thread
receive_thread = threading.Thread(target=receive_data, daemon=True)
receive_thread.start()

# Start Tkinter main loop
root.mainloop()
