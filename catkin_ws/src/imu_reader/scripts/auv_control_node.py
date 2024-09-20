#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def accel_callback(data):
    rospy.loginfo("Received accelerometer data: %s", data.data)
    # Process accelerometer data
    # acc_x, acc_y, acc_z = data.data

def gyro_callback(data):
    rospy.loginfo("Received gyroscope data: %s", data.data)
    # Process gyroscope data
    # gyro_x, gyro_y, gyro_z = data.data

def auv_control():
    rospy.init_node('auv_control_node', anonymous=True)
    rospy.Subscriber('accel_data', Float32MultiArray, accel_callback)
    rospy.Subscriber('gyro_data', Float32MultiArray, gyro_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        auv_control()
    except rospy.ROSInterruptException:
        pass
