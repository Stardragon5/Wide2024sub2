#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Accel
import os
import time
import board
import adafruit_icm20x

i2c = board.I2C()  # uses board.SCL and board.SDA
icm = adafruit_icm20x.ICM20649(i2c)

def icm20649_publisher():
    rospy.init_node('icm20649_node', anonymous=True)

    home_directory = os.path.expanduser("~")
    log_file_path = os.path.join(home_directory, "imu_log.txt")
    log_file = open(log_file_path, "w")

    imu_pub = rospy.Publisher('imu_out', Accel, queue_size=10)
    imu_msg = Accel()

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        imu_msg.linear.x = icm.acceleration[0]
        imu_msg.linear.y = icm.acceleration[1]
        imu_msg.linear.z = icm.acceleration[2]
        
        imu_msg.angular.x = icm.gyro[0]
        imu_msg.angular.y = icm.gyro[1]
        imu_msg.angular.z = icm.gyro[2]

        # Write to the log file
        log_entry = f"Acceleration: X:{imu_msg.linear.x:.2f}, Y:{imu_msg.linear.y:.2f}, Z:{imu_msg.linear.z:.2f} m/s^2\n"
        log_entry += f"Gyro: X:{imu_msg.angular.x:.2f}, Y:{imu_msg.angular.y:.2f}, Z:{imu_msg.angular.z:.2f} rads/s\n\n"
        log_file.write(log_entry)

        imu_pub.publish(imu_msg)
        rate.sleep()

    log_file.close()

if __name__ == '__main__':
    try:
        icm20649_publisher()
    except rospy.ROSInterruptException:
        pass
