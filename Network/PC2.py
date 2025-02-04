import cv2
import socket
import pickle
import struct
import tkinter as tk
from tkinter import Label, Button
from PIL import Image, ImageTk
import os

# Server Connection Settings
host_ip = '192.168.171.250'  # Your PC's IP address
port = 9999

# Create the main Tkinter window
root = tk.Tk()
root.title("Camera Feed")
root.geometry("800x600")

# Create a label to display the camera feed
video_label = Label(root)
video_label.pack()

# Connect to client (Raspberry Pi or another sender)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.bind((host_ip, port))
client_socket.listen(5)
print(f"Listening on {host_ip}:{port}")

client_conn, addr = client_socket.accept()
print(f"Connection from: {addr}")

data = b""
payload_size = struct.calcsize("L")
last_frame = None  # Store the last received frame globally


def receive_frame():
    """Receives a frame from the socket and updates the Tkinter GUI."""
    global data, last_frame

    while len(data) < payload_size:
        data += client_conn.recv(4096)
    
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("L", packed_msg_size)[0]

    while len(data) < msg_size:
        data += client_conn.recv(4096)
    
    frame_data = data[:msg_size]
    data = data[msg_size:]

    # Deserialize the frame and store it globally
    last_frame = pickle.loads(frame_data)

    # Convert the frame to ImageTk format
    cv_image = cv2.cvtColor(last_frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(cv_image)
    imgtk = ImageTk.PhotoImage(image=img)

    # Update the video label with the new frame
    video_label.imgtk = imgtk
    video_label.configure(image=imgtk)

    # Schedule the next frame update
    root.after(10, receive_frame)


def save_image():
    """Saves the last received frame as a unique image file."""
    global last_frame

    if last_frame is None:
        print("No frame received yet.")
        return

    # Create the output directory if it doesn't exist
    image_dir = "captured_images"
    os.makedirs(image_dir, exist_ok=True)

    # Get the next unique filename
    file_count = len([name for name in os.listdir(image_dir) if name.startswith("image_") and name.endswith(".jpg")])
    filename = os.path.join(image_dir, f"image_{file_count + 1:03d}.jpg")

    # Save the frame as an image
    cv2.imwrite(filename, last_frame)
    print(f"Image saved as: {filename}")


def stop_stream():
    """Stops the camera stream and closes the application."""
    client_conn.close()
    client_socket.close()
    root.quit()


# Create buttons for taking pictures and stopping the stream
capture_button = Button(root, text="Capture Image", command=save_image, font=("Arial", 14), bg="green", fg="white")
capture_button.pack(pady=10)

stop_button = Button(root, text="Stop Stream", command=stop_stream, font=("Arial", 14), bg="red", fg="white")
stop_button.pack(pady=10)

# Start receiving frames
receive_frame()

# Run the Tkinter main loop
root.mainloop()
