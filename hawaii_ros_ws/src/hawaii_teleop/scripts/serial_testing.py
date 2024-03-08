# import serial
# import time

# # Open serial connection
# ser = serial.Serial('/dev/ttyACM0', 250000, timeout=5)
# time.sleep(2)  # Allow time for the connection to initialize

# # Send the 'I am ready' message to the Arduino
# # print(ser.write(b'I am ready\n'))

# # Continuously read and process incoming messages

# # try:
# #     ser.readline()
# # except serial.SerialException as e:
# #     print("Error writing to serial:", e)
# #     ser.close()
# #     exit(1)
# try:
#     while True:
#         if ser.in_waiting > 0:
#             incomingData = ser.readline().decode('utf-8').strip()  # Read a line and strip trailing newline
#             print(f"Received: {incomingData}")
# except KeyboardInterrupt:
#     print("Program interrupted by user. Closing serial connection.")
# finally:
#     ser.close()  # Ensure the serial connection is closed on exit

import serial
import time

def read_from_teensy():
    # Serial port configuration
    port = "/dev/ttyACM0"
    baud_rate = 250000

    try:
        # Initialize serial connection
        ser = serial.Serial(port, baud_rate)

        # Check if the port is open
        if not ser.is_open:
            raise IOError("Serial port failed to open.")

        # Flush the buffer
        ser.flush()

        while True:
            if ser.in_waiting > 0:
                # Read data from serial port
                line = ser.readline().decode('utf-8').rstrip()
                # Print the received data
                print(line)

    except KeyboardInterrupt:
        # Gracefully exit on Ctrl+C and close serial connection
        print("Exiting...")
        ser.close()

    except Exception as e:
        # Handle other exceptions and close serial connection
        print(f"Error: {e}")
        ser.close()

if __name__ == "__main__":
    read_from_teensy()
