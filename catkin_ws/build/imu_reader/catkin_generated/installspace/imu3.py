#!/usr/bin/env python3

import rospy
#import smbus2
import os
import board
import adafruit_icm20x
from geometry_msgs.msg import Accel



i2c = board.I2C()  # uses board.SCL and board.SDA
icm = adafruit_icm20x.ICM20649(i2c)

# ICM20649 Registers
PWR_MGMT_1 = 0x06
ACCEL_XOUT_H = 0x2D
GYRO_XOUT_H = 0x33

# class ICM20649:
#     def __init__(self, bus=1, address=0x68):
#         self.bus = smbus2.SMBus(bus)
#         self.address = address
#         self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
#         time.sleep(0.1)

#     def read_raw_data(self, addr):
#         high = self.bus.read_byte_data(self.address, addr)
#         low = self.bus.read_byte_data(self.address, addr+1)
#         value = ((high << 8) | low)
#         if(value > 32768):
#             value = value - 65536
#         return value

#     def get_accel_data(self):
#         acc_x = self.read_raw_data(ACCEL_XOUT_H)
#         acc_y = self.read_raw_data(ACCEL_XOUT_H+2)
#         acc_z = self.read_raw_data(ACCEL_XOUT_H+4)
#         return [acc_x, acc_y, acc_z]

#     def get_gyro_data(self):
#         gyro_x = self.read_raw_data(GYRO_XOUT_H)
#         gyro_y = self.read_raw_data(GYRO_XOUT_H+2)
#         gyro_z = self.read_raw_data(GYRO_XOUT_H+4)
#         return [gyro_x, gyro_y, gyro_z]

def icm20649_publisher():
    rospy.init_node('icm20649_node', anonymous=True)

    home_directory = os.path.expanduser("~")
    log_file_path = os.path.join(home_directory, "imu_log.txt")
    log_file = open(log_file_path, "a")

    imu_pub = rospy.Publisher('imu_out', Accel, queue_size=10)
    imu_msg = Accel()
    
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
#        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (icm.acceleration))
 #       print("Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (icm.gyro))
  #      print("")
        imu_msg.linear.x = icm.acceleration[0]
        imu_msg.linear.y = icm.acceleration[1]
        imu_msg.linear.z = icm.acceleration[2]
        
        imu_msg.angular.x = icm.gyro[0]
        imu_msg.angular.y = icm.gyro[1]
        imu_msg.angular.y = icm.gyro[2]

        imu_pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        icm20649_publisher()
    except rospy.ROSInterruptException:
        pass
