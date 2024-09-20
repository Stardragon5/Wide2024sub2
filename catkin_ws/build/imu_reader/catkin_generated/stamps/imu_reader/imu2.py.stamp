#!/usr/bin/env python3

import rospy
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from std_msgs.msg import Float32MultiArray

def read_imu():
    # Create the I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)
    # Create the ADS object
    ads = ADS.ADS1115(i2c)
    # Create single-ended input channels
    chan0 = AnalogIn(ads, ADS.P0)
    chan1 = AnalogIn(ads, ADS.P1)
    chan2 = AnalogIn(ads, ADS.P2)
    chan3 = AnalogIn(ads, ADS.P3)

    # Initialize ROS node
    rospy.init_node('imu_reader', anonymous=True)
    imu_pub = rospy.Publisher('imu_data', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Read raw ADC values
        raw_values = [
            chan0.value,
            chan1.value,
            chan2.value,
            chan3.value
        ]
        rospy.loginfo(f"Raw ADC values: {raw_values}")

        # Scale the raw values to the appropriate range
        scaled_values = [value / 32768.0 * 16.0 for value in raw_values]
        rospy.loginfo(f"Scaled IMU values: {scaled_values}")

        # Publish the scaled values
        imu_msg = Float32MultiArray(data=scaled_values)
        imu_pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        read_imu()
    except rospy.ROSInterruptException:
        pass
