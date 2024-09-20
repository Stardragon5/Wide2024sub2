#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

class AUVControl:
    def __init__(self):
        rospy.init_node('auv_control_node', anonymous=True)
        rospy.Subscriber('accel_data', Float32MultiArray, self.accel_callback)
        rospy.Subscriber('gyro_data', Float32MultiArray, self.gyro_callback)
        self.accel_data = [0, 0, 0]
        self.gyro_data = [0, 0, 0]
        rospy.spin()

    def accel_callback(self, data):
        rospy.loginfo("Received accelerometer data: %s", data.data)
        self.accel_data = data.data
        self.process_data()

    def gyro_callback(self, data):
        rospy.loginfo("Received gyroscope data: %s", data.data)
        self.gyro_data = data.data
        self.process_data()

    def process_data(self):
        # Example processing function
        acc_x, acc_y, acc_z = self.accel_data
        gyro_x, gyro_y, gyro_z = self.gyro_data

        # Perform any data processing or control algorithms here
        rospy.loginfo("Processing data:")
        rospy.loginfo("Accel: x=%f, y=%f, z=%f" % (acc_x, acc_y, acc_z))
        rospy.loginfo("Gyro: x=%f, y=%f, z=%f" % (gyro_x, gyro_y, gyro_z))

        # Example: Simple threshold check
        if abs(acc_x) > 1000 or abs(acc_y) > 1000 or abs(acc_z) > 1000:
            rospy.logwarn("Accelerometer threshold exceeded!")

        if abs(gyro_x) > 5000 or abs(gyro_y) > 5000 or abs(gyro_z) > 5000:
            rospy.logwarn("Gyroscope threshold exceeded!")

if __name__ == '__main__':
    try:
        AUVControl()
    except rospy.ROSInterruptException:
        pass
