import ms5837
import rospy
from std_msgs.msg import Float64


rospy.init_node("depth")


# Create an instance of the MS5837 class
sensor = ms5837.MS5837_30BA()  # For Bar30 sensor
# sensor = ms5837.MS5837_02BA()  # For Bar02 sensor

# Initialize the sensor
if not sensor.init():
    print("Sensor could not be initialized")
    exit(1)

# Set fluid density to that of seawater (kg/m^3)
sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)  # Use ms5837.DENSITY_FRESHWATER for fresh water

# Read sensor data
if not sensor.read():
    print("Sensor read failed")
    exit(1)

# Print the data
print(f"Pressure: {sensor.pressure(ms5837.UNITS_mbar):.2f} mbar")
print(f"Temperature: {sensor.temperature(ms5837.UNITS_Centigrade):.2f} C")
print(f"Depth: {sensor.depth():.2f} m")

depthPub = rospy.Publisher('depthm', Float64, queue_size=10)


# Continuously read data
try:
    while True:
        if sensor.read():
            print(f"Depth: {sensor.depth():.2f} m, Pressure: {sensor.pressure(ms5837.UNITS_mbar):.2f} mbar, Temperature: {sensor.temperature(ms5837.UNITS_Centigrade):.2f} C")
            depthPub.publish(sensor.depth())
        else:
            print("Sensor read failed")
        rospy.sleep(1)
except KeyboardInterrupt:
    print("Stopping data read")
