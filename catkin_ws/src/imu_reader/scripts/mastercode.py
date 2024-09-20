import cv2 

import rospy 

from std_msgs.msg import Int32 

import numpy as np 

from adafruit_servokit import ServoKit 

import time 

import board 

import busio 

import adafruit_pca9685 

 

# Initialize ROS node 

rospy.init_node("color_motor_control") 

 

# Initialize the motor kit 

hat = ServoKit(channels=16) 

 

# Define motors (Looking from the front) 

back_left = hat.continuous_servo[12] 

left_vert = hat.continuous_servo[11] 

right_vert = hat.continuous_servo[10] 

back_right = hat.continuous_servo[13] 

front_right = hat.continuous_servo[14] 

front_left = hat.continuous_servo[15] 

thrusters = [back_left, left_vert, right_vert, back_right, front_right, front_left] 

 

# Set pulse range for the motors 

for motor in thrusters: 

    motor.set_pulse_width_range(1100, 1900) 

 

# Initialize all motors to a default state (e.g., stop) 

def initialize_motors(): 

    rospy.loginfo("Initializing motors") 

    stop_motors() 

 

# Function to stop all motors 

def stop_motors(): 

    for motor in thrusters: 

        motor.throttle = 0.1 

    rospy.loginfo("Motors stopped") 

 

# Function to move all motors forward 

def move_forward(): 

    for motor in thrusters: 

        motor.throttle = 0.5  # Adjust the throttle as needed for your setup 

    rospy.loginfo("Moving forward") 

 

# Function to increase vertical thruster speeds 

def increase_vertical_thrust(): 

    step = 0.1  # Step size for increasing speed 

    left_vert.throttle = min(left_vert.throttle + step, 1.0)  # Limit to a maximum throttle of 1.0 

    right_vert.throttle = min(right_vert.throttle + step, 1.0) 

    rospy.loginfo(f"Increasing vertical thrust: left_vert={left_vert.throttle}, right_vert={right_vert.throttle}") 

 

# Function to decrease vertical thruster speeds 

def decrease_vertical_thrust(): 

    step = 0.1  # Step size for decreasing speed 

    left_vert.throttle = max(left_vert.throttle - step, -1.0)  # Limit to a minimum throttle of -1.0 

    right_vert.throttle = max(right_vert.throttle - step, -1.0) 

    rospy.loginfo(f"Decreasing vertical thrust: left_vert={left_vert.throttle}, right_vert={right_vert.throttle}") 

 

# Initialize PCA9685 

i2c = busio.I2C(board.SCL, board.SDA) 

hat = adafruit_pca9685.PCA9685(i2c) 

hat.frequency = 54  # may need to check this 

 

# Define motors for PCA9685 

motor_front_left = hat.channels[14] 

motor_front_right = hat.channels[15] 

motor_up_left = hat.channels[10] 

motor_up_right = hat.channels[11] 

motor_rear_left = hat.channels[13] 

motor_rear_right = hat.channels[12] 

 

def calc_cycle(throttle):  # throttle is in microseconds 

    period = 1 / hat.frequency 

    duty = throttle / period 

    return duty 

 

def all_off(): 

    rospy.loginfo('All motors off') 

    throttle = 1500 

    motor_front_left.duty_cycle = calc_cycle(throttle) 

    motor_front_right.duty_cycle = calc_cycle(throttle) 

    motor_rear_left.duty_cycle = calc_cycle(throttle) 

    motor_rear_right.duty_cycle = calc_cycle(throttle) 

    time.sleep(0.2) 

 

def fwd_thrust(dur_sec, throttle): 

    rospy.loginfo('Forward thrust') 

    while dur_sec > 0: 

        motor_front_left.duty_cycle = calc_cycle(throttle) 

        motor_front_right.duty_cycle = calc_cycle(throttle) 

        motor_rear_left.duty_cycle = calc_cycle(throttle) 

        motor_rear_right.duty_cycle = calc_cycle(throttle) 

        dur_sec -= 0.02 

        time.sleep(0.02) 

    all_off() 

 

# Placeholder function to get the position of the vehicle using IMU 

def get_vehicle_position(): 

    # This function should return the current position of the vehicle 

    # Example: return [x, y, z] 

    return [0, 0, 0] 

 

# Placeholder function to translate the position of the vehicle 

def translate_position(position): 

    # Translate the position using IMU data 

    # Example: return translated_position 

    return position 

 

# Color publisher 

color_pub = rospy.Publisher('camera_color', Int32, queue_size=10) 

 

# Get parameter values 

frequency = rospy.get_param('~/frequency', 2) 

 

# Webcamera no 0 is used to capture the frames 

cap = cv2.VideoCapture(0) 

 

# Set variables 

rate = rospy.Rate(frequency) 

color = 0 

 

# State variable to keep track of motor status 

moving_forward = False 

 

# Call the initialization function at the start of the program 

initialize_motors() 

 

# Data collection loop 

while not rospy.is_shutdown(): 

    # Capture frame-by-frame 

    ret, frame = cap.read() 

    if not ret: 

        rospy.logwarn("Failed to capture frame") 

        continue 

 

    # Create a downscaled frame 

    size = frame.shape 

    frame1 = np.zeros((int(size[0] / 10), int(size[1] / 10), size[2]), dtype=np.uint8) 

    for i in range(0, int(size[0] / 10)): 

        for j in range(0, int(size[1] / 10)): 

            frame1[i, j] = frame[10 * i, 10 * j] 

 

    # Define color BGR ranges 

    color_ranges = { 

        "red": ([9, 13, 112], [41, 35, 171], 1), 

        "orange": ([7, 40, 136], [49, 104, 193], 2), 

        "yellow": ([10, 100, 134], [46, 152, 186], 3), 

        "green": ([95, 100, 29], [160, 181, 109], 4), 

        "blue": ([90, 50, 20], [150, 120, 80], 5), 

        "purple": ([80, 30, 40], [152, 84, 128], 6), 

        "black": ([24, 19, 17], [70, 70, 60], 7) 

    } 

 

    # Calculate dem 

    dem = 255 * frame1.shape[0] * frame1.shape[1] 

 

    # Threshold for color detection 

    thresh = 0.2 

 

    max_reading = 0 

    detected_color = None 

    detected_color_value = None 

 

    # Find color with the highest reading 

    for color_name, (lower, upper, color_value) in color_ranges.items(): 

        mask = cv2.inRange(frame1, np.array(lower), np.array(upper)) 

        color_t = np.sum(mask) / dem 

        rospy.loginfo(f'{color_name.capitalize()}: {color_t}') 

 

        if color_t > max_reading: 

            max_reading = color_t 

            detected_color = color_name 

            detected_color_value = color_value 

 

    # Determine actions based on detected color 

    if detected_color == "orange" and max_reading > thresh and not moving_forward: 

        rospy.loginfo(f"Detected orange with reading {max_reading}") 

        move_forward() 

        moving_forward = True  # Update state to indicate moving forward 

    elif detected_color == "purple" and max_reading > thresh and moving_forward: 

        rospy.loginfo(f"Detected purple with reading {max_reading}") 

        stop_motors() 

        moving_forward = False  # Update state to indicate motors are stopped 

    elif detected_color == "yellow" and max_reading > thresh: 

        # Perform operations only if yellow has the highest reading 

        rospy.loginfo(f"Detected yellow with reading {max_reading}") 

        decrease_vertical_thrust() 

    elif detected_color == "green" and max_reading > thresh: 

        # Perform operations only if green has the highest reading 

        rospy.loginfo(f"Detected green with reading {max_reading}") 

        increase_vertical_thrust() 

    else: 

        rospy.loginfo("No relevant color detected above the threshold") 

        if moving_forward: 

            # Keep moving forward if currently doing so 

            rospy.loginfo("Continuing to move forward") 

 

    # Publish detected color 

    if detected_color_value is not None: 

        color_pub.publish(detected_color_value) 

    else: 

        color_pub.publish(0) 

 

    # Rotate frame and handle any interruptions 

    frame1 = cv2.rotate(frame1, cv2.ROTATE_180) 

    # cv2.imshow('frame1', frame1) 

 

    # Break loop if the ESC key is pressed 

    k = cv2.waitKey(5) & 0xFF 

    if k == 27: 

        break 

 

    # Rate sleep 

    rate.sleep() 