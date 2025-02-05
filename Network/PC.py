import cv2
import socket
import pickle
import struct
import tkinter as tk
from tkinter import Button, Entry
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

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

# Initialize Tkinter window
root = tk.Tk()
root.title("Raspberry Pi Video Feed")

# Create a canvas for video feed
video_frame = tk.Label(root)
video_frame.pack()

# Create an entry for speed input
speed_entry = Entry(root)
speed_entry.pack()

# Create buttons
def send_command(command):
    client_socket.sendall(command.encode('utf-8'))

def take_picture():
    # Logic to take a picture (not implemented in this example)
    print("Picture taken!")

start_stop_button = Button(root, text="Start/Stop", command=lambda: send_command("start" if start_stop_button['text'] == "Start" else "stop"))
start_stop_button.pack()

take_picture_button = Button(root, text="Take Picture", command=take_picture)
take_picture_button.pack()

# Create a Matplotlib figure for 3D points
fig = plt.Figure()
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# Function to update video feed and plot
def update():
    global data
    # Send speed command
    speed = speed_entry.get()
    if speed:
        send_command(f"speed:{speed}")

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

    # Convert frame to ImageTk format and display
    img = Image.fromarray(frame)
    imgtk = ImageTk.PhotoImage(image=img)
    video_frame.imgtk = imgtk
    video_frame.configure(image=imgtk)

    # Handle incoming 3D points
    points_data = data  # Assuming points data follows frame data
    points_3d = pickle.loads(points_data)
    ax.clear()
    ax.scatter(*zip(*points_3d))  # Unpack points for plotting
    canvas.draw()

    # Schedule the next update
    root.after(10, update)

# Start the update loop
update()

# Start the Tkinter main loop
root.mainloop()

# Cleanup
client_socket.close()
server_socket.close()
