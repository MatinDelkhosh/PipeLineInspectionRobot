import tensorflow.lite as tflite # for windows
#import tensorflow.lite.Interpreter as tflite #for raspberry pi
import numpy as np
from PIL import Image, ImageDraw
import os

def load_labels(path):
    with open(path, 'r') as f:
        return {i: line.strip() for i, line in enumerate(f.readlines())}

def preprocess_image(image_path, input_shape):
    # Open the image and convert it to RGB format
    image = Image.open(image_path).convert('RGB')
    
    # Ensure input_shape is a tuple of two integers
    if isinstance(input_shape, (np.ndarray, list)):
        input_shape = tuple(int(x) for x in input_shape[:2])  # Take the first two elements and convert to integers
    elif not isinstance(input_shape, tuple) or len(input_shape) != 2:
        raise ValueError("input_shape must be a tuple of two integers (width, height)")
    
    # Resize the image to the specified input shape using LANCZOS resampling
    image = image.resize(input_shape, Image.LANCZOS)
    
    # Convert the image to a numpy array and normalize to [0, 1] range
    input_data = np.array(image, dtype=np.float32) / 255.0  # Normalize to FLOAT32
    
    # Expand dimensions to match model input shape (e.g., [1, height, width, 3])
    input_data = np.expand_dims(input_data, axis=0)
    
    return image, input_data

def draw_keypoints(image, keypoints, threshold=0.5):
    draw = ImageDraw.Draw(image)
    for kp in keypoints:
        if kp[2] > threshold:
            x, y = kp[0], kp[1]
            draw.ellipse([(x - 2, y - 2), (x + 2, y + 2)], fill='red')
    return image

def detect(image_path):
    # Load TFLite model and allocate tensors
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, 'models', 'CenterNet_MobileNet_V2_Keypoints.tflite')  # Path to the TFLite model
    
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found at: {model_path}")
    
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    # Get input and output tensors
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Model expects image size, extract from input_details
    input_shape = input_details[0]['shape'][1:3]

    # Load and preprocess the image
    image_path = os.path.join(script_dir,image_path)
    image, input_data = preprocess_image(image_path, input_shape)

    # Set the tensor for input data
    interpreter.set_tensor(input_details[0]['index'], input_data)

    # Run inference
    interpreter.invoke()

    # Extract keypoints from the output tensors
    keypoints = interpreter.get_tensor(output_details[0]['index'])[0]  # Adjust based on model's output format

    # Visualize results
    result_image = draw_keypoints(image, keypoints)
    result_image.show()  # Displays the image with keypoints

detect('image2.jpg')