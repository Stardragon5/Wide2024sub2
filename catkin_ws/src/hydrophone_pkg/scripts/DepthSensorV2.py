import ms5837
import rospy
from std_msgs.msg import Float64
import os

rospy.init_node("depth")

# Create an instance of the MS5837 class
sensor = ms5837.MS5837_30BA()  # For Bar30 sensor
# sensor = ms5837.MS5837_02BA()  # For Bar02 sensor

# Initialize the sensor
if not sensor.init():
    rospy.logerr("Sensor could not be initialized")
    exit(1)

# Set fluid density to that of seawater (kg/m^3)
sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)  # Use ms5837.DENSITY_FRESHWATER for fresh water

# Read sensor data
if not sensor.read():
    rospy.logerr("Sensor read failed")
    exit(1)

# Print the data
#rospy.loginfo(f"Pressure: {sensor.pressure(ms5837.UNITS_mbar):.2f} mbar")
#rospy.loginfo(f"Temperature: {sensor.temperature(ms5837.UNITS_Centigrade):.2f} C")
#rospy.loginfo(f"Depth: {sensor.depth():.2f} m")

depthPub = rospy.Publisher('depthm', Float64, queue_size=10)

# Open a file to log the depth data
home_directory = os.path.expanduser("~")
log_file_path = os.path.join(home_directory, "depth_log.txt")
log_file = open(log_file_path, "w")

# Continuously read data
rate = rospy.Rate(1)  # 1 Hz
try:
    while not rospy.is_shutdown():
        if sensor.read():
            depth = sensor.depth()
            pressure = sensor.pressure(ms5837.UNITS_mbar)
            temperature = sensor.temperature(ms5837.UNITS_Centigrade)
            
#            rospy.loginfo(f"Depth: {depth:.2f} m, Pressure: {pressure:.2f} mbar, Temperature: {temperature:.2f} C")
            depthPub.publish(depth)
            
            # Log the data to the file
            log_file.write(f"Depth: {depth:.2f} m, Pressure: {pressure:.2f} mbar, Temperature: {temperature:.2f} C\n")
        else:
            rospy.logerr("Sensor read failed")
        rate.sleep()
except rospy.ROSInterruptException:
    rospy.loginfo("Stopping data read")
finally:
    log_file.close()
    rospy.signal_shutdown("Node is shutting down")
