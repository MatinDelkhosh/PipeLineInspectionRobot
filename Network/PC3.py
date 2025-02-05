import cv2
import socket
import pickle
import struct
import tkinter as tk
from tkinter import Button
from PIL import Image, ImageTk
import os
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

# Function to save the current frame
def save_image():
    global current_frame
    if current_frame is not None:
        # Generate a unique filename using the current timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"captured_image_{timestamp}.jpg"
        cv2.imwrite(filename, current_frame)
        print(f"Image saved as {filename}")

# Create a button to save the image
save_button = Button(root, text="Save Image", command=save_image)
save_button.pack()

# Global variable to store the current frame
current_frame = None

# Function to update the video feed
def update_video_feed():
    global current_frame
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
            current_frame = frame  # Store the current frame for saving

            # Convert the frame to RGB for display in Tkinter
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)

            # Update the video feed label
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)

            # Schedule the next frame update
            video_label.after(10, update_video_feed)

    except Exception as e:
        print(f"Error: {e}")

# Start the video feed update loop
update_video_feed()

# Start the Tkinter main loop
root.mainloop()

# Cleanup
client_socket.close()
server_socket.close()