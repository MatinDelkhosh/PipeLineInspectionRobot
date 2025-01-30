import RPi.GPIO as GPIO
import time
import smbus
import math
#import matplotlib.pyplot as plt
from threading import Thread
#import cv2

# Define GPIO pins for motors and ultrasonic sensors
MOTOR_LEFT_FORWARD = 23   #ENA
MOTOR_LEFT_IN1 = 17  #IN1
MOTOR_RIGHT_FORWARD = 24  #ENA
MOTOR_RIGHT_IN1 = 27 #IN1

TRIG_LEFT = 5
ECHO_LEFT = 6
TRIG_RIGHT = 13
ECHO_RIGHT = 19

# Initialize GPIO pins
GPIO.setmode(GPIO.BCM)

GPIO.setup(MOTOR_LEFT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_IN1, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_IN1, GPIO.OUT)

GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

# PWM setup
motor_left_pwm = GPIO.PWM(MOTOR_LEFT_FORWARD, 100)
motor_right_pwm = GPIO.PWM(MOTOR_RIGHT_FORWARD, 100)
motor_left_pwm.start(0)
motor_right_pwm.start(0)

# MPU6050 setup
IMU_ADDRESS = 0x68
bus = smbus.SMBus(1)
bus.write_byte_data(IMU_ADDRESS, 0x6B, 0)

'''x, y = 0, 0  
trajectory = [(x, y)]  '''
wheel_diameter = 6.0  #  cm
encoder_resolution = 360  # ppr
wheel_circumference = math.pi * wheel_diameter

# Realtime plotting setup
'''plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], marker="o")
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.set_title("2D Trajectory Mapping")
ax.set_xlabel("X Position (cm)")
ax.set_ylabel("Y Position (cm)")
ax.grid()'''

def calculate_distance(encoder_pulses):
    return (encoder_pulses / encoder_resolution) * wheel_circumference

def update_position(x, y, distance, angle):
    x_new = x + distance * math.cos(math.radians(angle))
    y_new = y + distance * math.sin(math.radians(angle))
    return x_new, y_new

'''def update_plot():
    while True:
        x_coords, y_coords = zip(*trajectory)
        line.set_data(x_coords, y_coords)
        plt.draw()
        plt.pause(0.01)'''

'''def read_distance(trig, echo):
    """Measure distance using an ultrasonic sensor."""
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    while GPIO.input(echo) == 0:
        start_time = time.time()

    while GPIO.input(echo) == 1:
        end_time = time.time()

    duration = end_time - start_time
    distance = (duration * 34300) / 2  # Speed of sound = 34300 cm/s
    return distance'''

from UT import measure_distance as read_distance
from Center import pipe_center

def control_motors(left_speed, right_speed):
    """Control motor speeds."""
    if left_speed > 0:
        GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
        motor_left_pwm.ChangeDutyCycle(left_speed)
    else:
        GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
        motor_left_pwm.ChangeDutyCycle(0)

    if right_speed > 0:
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
        motor_right_pwm.ChangeDutyCycle(right_speed)
    else:
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
        motor_right_pwm.ChangeDutyCycle(0)

'''def read_imu():
    """Read IMU data (example using accelerometer)."""
    accel_x = bus.read_byte_data(IMU_ADDRESS, 0x3B)
    accel_y = bus.read_byte_data(IMU_ADDRESS, 0x3D)
    accel_z = bus.read_byte_data(IMU_ADDRESS, 0x3F)
    gyro_z = bus.read_byte_data(IMU_ADDRESS, 0x47)  # Example for Z-axis rotation
    return accel_x, accel_y, accel_z, gyro_z'''

from IMU import read_imu

def main():
    #global x, y, trajectory
    try:
        SENSITIVITY_THRESHOLD = 10  # Minimum difference to trigger motor adjustment
        TURN_FACTOR = 0.5  # Factor to adjust sharpness of turns

        # Start the realtime plotting thread
        '''plot_thread = Thread(target=update_plot)
        plot_thread.daemon = True
        plot_thread.start()'''


        #cap = cv2.VideoCapture(0)

        while True:
            # Read distances from ultrasonic sensors
            distance_left = read_distance(TRIG_LEFT, ECHO_LEFT)
            distance_right = read_distance(TRIG_RIGHT, ECHO_RIGHT)
            #centerofpipe = pipe_center(cap)
            #print(f'pipe center location: {centerofpipe}')

            # Calculate the difference between left and right distances
            distance_diff = distance_left - distance_right

            # Determine motor speeds based on distance difference
            if abs(distance_diff) < SENSITIVITY_THRESHOLD:
                left_speed = 60
                right_speed = 60
            else:
                adjustment = min(20, max(5, TURN_FACTOR * abs(distance_diff)))
                if distance_diff > 0:  # Turn right
                    left_speed = 60 + adjustment
                    right_speed = 60 - adjustment
                else:  # Turn left
                    left_speed = 60 - adjustment
                    right_speed = 60 + adjustment

            control_motors(left_speed, right_speed)

            # Read IMU and encoder data
            imu_data = read_imu()
            accel_x = imu_data[0]
            accel_y = imu_data[1]
            gyro_z = imu_data[3]
            encoder_pulses = 20

            # Calculate distance from encoder
            distance = calculate_distance(encoder_pulses)

            # Update position
            angle = gyro_z  # Example for using gyro for orientation
            x, y = update_position(x, y, distance, angle)

            # Save trajectory
            #trajectory.append((x, y))

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping robot...")
        motor_left_pwm.stop()
        motor_right_pwm.stop()
        #cap.release()
        #cv2.destroyAllWindows()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
