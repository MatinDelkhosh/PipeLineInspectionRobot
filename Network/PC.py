import cv2
import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import simpledialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading

# Create a socket to receive data
host = '192.168.171.150'  # Replace with Raspberry Pi's IP address
port = 9999  # Port used for communication
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('0.0.0.0', port))

# Initialize the OpenCV window to display the camera feed (we won't need this anymore)
# cv2.namedWindow('Camera Feed')

# Initialize the Tkinter root window for GUI
root = tk.Tk()
root.geometry('1200x800')  # Full screen or adjust the size

motor_state = tk.StringVar(value="Stopped")
base_speed = tk.IntVar(value=0)

# Create a socket and set up the plot
fig, ax = plt.subplots(1, 1, figsize=(12, 7))
ax.set_title("3D Plot with Camera Feed Overlay")

# Create a canvas to embed the plot into Tkinter
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Create a list of points
points = []

# Function to handle the button click (start/stop motor)
def on_button_click():
    global motor_state
    motor_state.set("Running" if motor_state.get() == "Stopped" else "Stopped")
    
    # Get user input for baseSpeed
    speed = simpledialog.askinteger("Motor Speed", "Enter baseSpeed:", parent=root, minvalue=0, maxvalue=100)
    
    # Send the motor state and baseSpeed to Raspberry Pi
    msg = f"{motor_state.get()}:{speed}".encode()
    sock.sendto(msg, (host, port))

# Function to update the 3D plot with new points
def update_points(new_points):
    """Update the 3D plot with new points"""
    global points
    points.extend(new_points)
    ax.clear()
    if points:
        ax.scatter(*zip(*points))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    canvas.draw()

# Function to receive data and update the display
def receive_data():
    """Receive frames and 3D points from Raspberry Pi"""
    while True:
        data, _ = sock.recvfrom(4096)
        if data.startswith(b'FRAME:'):
            frame_data = data[6:]
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                update_frame(frame)
        elif data.startswith(b'POINTS:'):
            points_data = data[7:].decode().strip()
            new_points = [tuple(map(float, point.split(','))) for point in points_data.split(';')]
            update_points(new_points)

# Function to update the frame in the matplotlib window
def update_frame(frame):
    """Update the camera feed in the plot"""
    # Convert OpenCV frame (BGR) to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    ax.imshow(frame_rgb, aspect='auto', extent=[-5, 5, -5, 5], alpha=0.6)  # Adjust extent for proper positioning
    canvas.draw()

# Start threads to receive data and update the plot
def start_threads():
    threading.Thread(target=receive_data, daemon=True).start()

# Tkinter button to control motor state and speed
button = tk.Button(root, text="Start/Stop Motor", command=on_button_click)
button.pack(pady=10)

# Start threads for receiving data from Raspberry Pi
start_threads()

# Start Tkinter GUI loop
root.mainloop()
