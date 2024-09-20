import csv
import time

# Simulating reading from an IMU sensor
def read_imu_data():
    # Replace this function with actual code to read from your IMU
    x = icm.acceleration[0]
    y = icm.acceleration[1]
    z = icm.acceleration[2]
    return x, y, z

# List to hold the IMU data
imu_data = []

# Number of readings to take
num_readings = 100

for _ in range(num_readings):
    x, y, z = read_imu_data()
    imu_data.append([x, y, z])
    time.sleep(0.1)  # Simulate a delay between readings

# Write the data to a CSV file
csv_file = 'imu_data.csv'
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Write the header
    writer.writerow(['X', 'Y', 'Z'])
    # Write the data
    writer.writerows(imu_data)

print(f"Data written to {csv_file}")
