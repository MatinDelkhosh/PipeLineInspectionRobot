import cv2
import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import simpledialog

# Create a socket to receive data
host = 'your_raspberry_pi_ip_address'  # Replace with Raspberry Pi's IP address
port = 12345  # Port used for communication
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', port))

# Initialize the plot for 3D points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
points = []

# Initialize OpenCV window to display the camera feed
cv2.namedWindow('Camera Feed')

# Tkinter window for user input
root = tk.Tk()
root.geometry('300x150')

motor_state = tk.StringVar(value="Stopped")
base_speed = tk.IntVar(value=0)

def on_button_click():
    global motor_state
    motor_state.set("Running" if motor_state.get() == "Stopped" else "Stopped")
    
    # Get user input for baseSpeed
    speed = simpledialog.askinteger("Motor Speed", "Enter baseSpeed:", parent=root, minvalue=0, maxvalue=100)
    
    # Send the motor state and baseSpeed to Raspberry Pi
    msg = f"{motor_state.get()}:{speed}".encode()
    sock.sendto(msg, (host, port))

def update_points(new_points):
    """Update the 3D plot with new points"""
    global points
    points.extend(new_points)
    ax.clear()
    ax.scatter(*zip(*points))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

def receive_data():
    """Receive frames and 3D points from Raspberry Pi"""
    while True:
        data, _ = sock.recvfrom(4096)
        if data.startswith(b'FRAME:'):
            frame_data = data[6:]
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('Camera Feed', frame)
        elif data.startswith(b'POINTS:'):
            points_data = data[7:].decode().strip()
            new_points = [tuple(map(float, point.split(','))) for point in points_data.split(';')]
            update_points(new_points)

def animate_plot():
    """Animate the 3D plot to reflect the latest data"""
    def update(frame):
        ax.clear()
        ax.scatter(*zip(*points))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        return ax,

    ani = FuncAnimation(fig, update, blit=False)
    plt.show()

# Button to control motor state and speed
button = tk.Button(root, text="Start/Stop Motor", command=on_button_click)
button.pack(pady=10)

# Start threads to receive data and update the plot
import threading
threading.Thread(target=receive_data, daemon=True).start()

# Start Tkinter GUI loop
root.mainloop()

# Start the plot animation loop
animate_plot()