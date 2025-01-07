import tensorflow.lite as tflite
#import tensorflow.lite.Interpreter as tflite
import numpy as np
from PIL import Image, ImageDraw

def load_labels(path):
    with open(path, 'r') as f:
        return {i: line.strip() for i, line in enumerate(f.readlines())}

def preprocess_image(image_path, input_shape):
    image = Image.open(image_path).convert('RGB').resize(input_shape, Image.ANTIALIAS)
    input_data = np.expand_dims(np.array(image, dtype=np.uint8), axis=0)
    return image, input_data

def draw_keypoints(image, keypoints, threshold=0.5):
    draw = ImageDraw.Draw(image)
    for kp in keypoints:
        if kp[2] > threshold:
            x, y = kp[0], kp[1]
            draw.ellipse([(x - 2, y - 2), (x + 2, y + 2)], fill='red')
    return image

def main():
    # Load TFLite model and allocate tensors
    model_path = 'models/CenterNet_MobileNet_V2_Keypoints.tflite'  # Path to the TFLite model
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    # Get input and output tensors
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Model expects image size, extract from input_details
    input_shape = input_details[0]['shape'][1:3]

    # Load and preprocess the image
    image_path = 'input_image.jpg'  # Replace with the path to your image
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

if __name__ == '__main__':
    main()