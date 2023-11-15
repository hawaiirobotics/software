import serial
import csv
import time

# Open serial port (Replace 'COM3' with your actual port and 9600 with your baud rate)
ser = serial.Serial('COM3', 9600, timeout=1)

# Create and open the CSV file
with open('data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "M1", "M2", "C1", "C2"])  # Write the header

    start_time = time.time()  # Record the start time

    while True:
        current_time = time.time()
        if (current_time - start_time) > 30:  # Check if 30 seconds have passed
            break

        try:
            line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port
            if line:
                data = line.split()  # Split the line into parts
                if len(data) == 4:
                    # Add a timestamp and write to the CSV
                    timestamp = int((current_time - start_time) * 1000)  # Current time in milliseconds
                    writer.writerow([timestamp] + data)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"An error occurred: {e}")

ser.close()  # Close the serial port
