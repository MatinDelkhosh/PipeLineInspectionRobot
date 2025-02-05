import sys
import cv2
import numpy as np
import tensorflow as tf

# Function to load the TensorFlow Lite model
def load_model(model_path):
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter

# Function to run inference on the model
def run_inference(interpreter, image_path):
    # Read the image
    image = cv2.imread(image_path)
    
    # Preprocess the image (resize, normalize, etc.)
    image_resized = cv2.resize(image, (224, 224))  # Adjust size to model input
    image_normalized = np.expand_dims(image_resized, axis=0).astype(np.float32) / 255.0

    # Get model input details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Set input tensor
    interpreter.set_tensor(input_details[0]['index'], image_normalized)

    # Run inference
    interpreter.invoke()

    # Get output
    output_data = interpreter.get_tensor(output_details[0]['index'])
    
    return output_data

def main():
    # The model path (adjust path if necessary)
    model_path = 'Detection/bump_detector.tflite'
    
    # Get the image path from the arguments
    image_path = sys.argv[1]
    
    # Load the model
    interpreter = load_model(model_path)
    
    # Run inference
    output_data = run_inference(interpreter, image_path)
    
    # Print or return the output data (for now, let's just print it)
    print(output_data)

if __name__ == '__main__':
    main()
