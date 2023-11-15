import csv
import numpy as np
import matplotlib.pyplot as plt

# Read the CSV file and parse it into an array
data = []
with open('data_noise.csv', 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Skip the header
    for row in csv_reader:
        data.append([int(row[0]), int(row[1]), int(row[2]), int(row[3]), int(row[4])])

# Convert list to a numpy array
data_array = np.array(data)

# Extracting individual columns
timestamps = data_array[:, 0]
M1_values = data_array[:, 1]
M2_values = data_array[:, 2]
C1_values = data_array[:, 3]
C2_values = data_array[:, 4]
C1_values_scaled = np.round(data_array[:, 3] / 2)
C2_values_scaled = np.round(data_array[:, 4] / 2)

# average
C_avg = (C1_values + C2_values) / 2
# scale the average
C_avg_scale = np.round(C_avg / 2)


# compare C1 to C2 (error)
C1_C2 = abs(C1_values - C2_values)
print("Capacitive Max Error: " + str(np.max(C1_C2)))
print("Capacitive Average Error: " + str(np.mean(C1_C2)))
print("Capacitive Std Dev Error: " + str(np.std(C1_C2)))


# compare M1 to M2 (error)
M1_M2 = abs(M1_values - M2_values)
print("Magnetic Max Error: " + str(np.max(M1_M2)))
print("Magnetic Average Error: " + str(np.mean(M1_M2)))
print("Magnetic Std Dev Error: " + str(np.std(M1_M2)))

# compare M1 to C1+C2 average
M1_C = abs(M1_values - C_avg_scale)
print("M1 to Capacitive Max Error: " + str(np.max(M1_C)))
print("M1 to Capacitive Average Error: " + str(np.mean(M1_C)))
print("M1 to Capacitive Std Dev Error: " + str(np.std(M1_C)))

# compare M2 to C1 + C2 average
M2_C = abs(M2_values - C_avg_scale)
print("M2 to Capacitive Max Error: " + str(np.max(M2_C)))
print("M2 to Capacitive Average Error: " + str(np.mean(M2_C)))
print("M2 to Capacitive Std Dev Error: " + str(np.std(M2_C)))



# Create subplots
fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

# Plotting the data on separate subplots
axs[0].plot(timestamps, M1_values, label='Mag 1', color='b')
axs[0].set_ylabel('M1 Values')
axs[0].legend(loc='upper right')

axs[1].plot(timestamps, M2_values, label='Mag 2', color='g')
axs[1].set_ylabel('M2 Values')
axs[1].legend(loc='upper right')

axs[2].plot(timestamps, C1_values, label='Capacitive 1', color='r')
axs[2].set_ylabel('C1 Values')
axs[2].legend(loc='upper right')

axs[3].plot(timestamps, C2_values, label='Capacitive 2', color='c')
axs[3].set_ylabel('C2 Values')
axs[3].set_xlabel('Timestamp (milliseconds)')
axs[3].legend(loc='upper right')

# Set the title for the top graph
axs[0].set_title('Plot of M1, M2, C1, C2 over Time')

plt.tight_layout()
plt.show()
