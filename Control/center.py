import cv2
import numpy as np

def detect_strongest_circle(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Enhance contrast
    gray = cv2.equalizeHist(gray)
    
    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(
        blurred, 
        cv2.HOUGH_GRADIENT, 
        dp=1.5, 
        minDist=30, 
        param1=80, 
        param2=30, 
        minRadius=20, 
        maxRadius=200
    )
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        strongest_circle = circles[0]
        x, y, r = strongest_circle
        
        # Get frame dimensions
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Calculate position relative to center
        relative_x = x - center_x
        relative_y = y - center_y

        # Draw the detected circle
        #cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
        #cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

        return (relative_x, relative_y), frame

    return None, frame

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))

if not cap.isOpened():
    print("Error: Could not open camera.")

def pipe_center(cap):
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        return (0,0)

    # Detect strongest circle and calculate relative position
    center_offset, output_frame = detect_strongest_circle(frame)

    if center_offset:
        return center_offset
    else:
        print("Error: Failed to recognize center")
        return (0,0)
'''
    # Show the processed frame
    cv2.imshow("Circle Detection", output_frame)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return (0,0)
'''
# Release resources
pipe_center(cap)
cap.release()
cv2.destroyAllWindows()