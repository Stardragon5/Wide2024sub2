import rospy
from std_msgs.msg import Float32
from smbus2 import SMBus
import numpy as np
from scipy.fft import fft

# Constants
ADC_ADDRESS = 0x48  # Replace with your ADC's I2C address
SAMPLE_RATE = 1000  # Sample rate in Hz
BUFFER_SIZE = 1024  # Number of samples in each buffer
PING_FREQUENCY = 400  # Expected ping frequency in Hz
PING_THRESHOLD = 10  # Amplitude threshold for ping detection

# Initialize the I2C bus
bus = SMBus(1)  # For Raspberry Pi 3 and later

def read_adc():
    # Implement the reading from the ADC connected to the hydrophone
    # This is a placeholder implementation and should be replaced with actual ADC reading code
    try:
        # Read data from ADC (depends on specific ADC and sensor setup)
        raw_data = bus.read_byte(ADC_ADDRESS)
        voltage = raw_data * (3.3 / 255)  # Assuming a 3.3V reference and 8-bit resolution
        return voltage
    except Exception as e:
        rospy.logerr(f"Error reading ADC: {e}")
        return None

def detect_ping(signal):
    # Apply FFT to the signal
    fft_result = fft(signal)
    # Calculate the magnitude spectrum
    magnitude = np.abs(fft_result)
    # Find the index corresponding to the PING_FREQUENCY
    freqs = np.fft.fftfreq(len(signal), 1/SAMPLE_RATE)
    target_index = np.argmax(magnitude[np.where((freqs >= PING_FREQUENCY - 10) & (freqs <= PING_FREQUENCY + 10))])
    target_amplitude = magnitude[target_index]
    # Check if the amplitude exceeds the threshold
    if target_amplitude > PING_THRESHOLD:
        return True, target_amplitude
    return False, 0

def hydrophone_node():
    rospy.init_node('hydrophone_node', anonymous=True)
    pub = rospy.Publisher('ping_detected', Float32, queue_size=10)
    rate = rospy.Rate(SAMPLE_RATE / BUFFER_SIZE)  # Control the loop frequency
    
    buffer = np.zeros(BUFFER_SIZE)
    while not rospy.is_shutdown():
        for i in range(BUFFER_SIZE):
            buffer[i] = read_adc()
        
        ping_detected, amplitude = detect_ping(buffer)
        if ping_detected:
            rospy.loginfo(f"Ping detected with amplitude: {amplitude}")
            pub.publish(amplitude)
        rate.sleep()

if __name__ == '__main__':
    try:
        hydrophone_node()
    except rospy.ROSInterruptException:
        pass

