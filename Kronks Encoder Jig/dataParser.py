import serial
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv

# Configuration
PORT = 'COM3'  # Change this to your Arduino's serial port
BAUD_RATE = 9600
CSV_FILENAME = "encoder_data.csv"

# Initialize serial connection
ser = serial.Serial(PORT, BAUD_RATE)
ser.flush()

# Set up matplotlib
fig, ax = plt.subplots()
xdata, c1data, c2data, m1data, m2data = [], [], [], [], []
ln1, = plt.plot([], [], 'r-', label="C1")
ln2, = plt.plot([], [], 'g-', label="C2")
ln3, = plt.plot([], [], 'b-', label="M1")
ln4, = plt.plot([], [], 'y-', label="M2")

c1_offset, c2_offset, m1_offset, m2_offset = None, None, None, None

def init():
    ax.set_xlim(0, 50)
    ax.set_ylim(0, 1024)  # assuming encoder ticks range from 0 to 1024, adjust if necessary
    ax.legend(loc="upper left")
    return ln1, ln2, ln3, ln4

def update(frame):
    ser_bytes = ser.readline().decode('utf-8').strip()
    # print(ser_bytes)
    encoder, ticks = ser_bytes.split(' ')
    ticks = int(ticks)

    global c1_offset, c2_offset, m1_offset, m2_offset

    # Set the offset for each encoder
    if encoder == "C1" and c1_offset is None:
        c1_offset = ticks
    elif encoder == "C2" and c2_offset is None:
        c2_offset = ticks
    elif encoder == "M1" and m1_offset is None:
        m1_offset = ticks
    elif encoder == "M2" and m2_offset is None:
        m2_offset = ticks

    # Subtract the offset from the tick value
    if encoder == "C1":
        ticks -= c1_offset
    elif encoder == "C2":
        ticks -= c2_offset
    elif encoder == "M1":
        ticks -= m1_offset
    elif encoder == "M2":
        ticks -= m2_offset
    else: 
        return ln1, ln2, ln3, ln4

    # ... Rest of the update function remains the same ...

    if len(xdata) < 50:  # adjust this if you want more or fewer data points on the graph
        xdata.append(len(xdata))
    else:
        c1data.pop(0)
        c2data.pop(0)
        m1data.pop(0)
        m2data.pop(0)

    if encoder == "C1":
        c1data.append(ticks)
        c2data.append(c2data[-1] if c2data else 0)
        m1data.append(m1data[-1] if m1data else 0)
        m2data.append(m2data[-1] if m2data else 0)
    elif encoder == "C2":
        c1data.append(c1data[-1] if c1data else 0)
        c2data.append(ticks)
        m1data.append(m1data[-1] if m1data else 0)
        m2data.append(m2data[-1] if m2data else 0)
    elif encoder == "M1":
        c1data.append(c1data[-1] if c1data else 0)
        c2data.append(c2data[-1] if c2data else 0)
        m1data.append(ticks)
        m2data.append(m2data[-1] if m2data else 0)
    elif encoder == "M2":
        c1data.append(c1data[-1] if c1data else 0)
        c2data.append(c2data[-1] if c2data else 0)
        m1data.append(m1data[-1] if m1data else 0)
        m2data.append(ticks)


    assert len(c1data) == len(c2data) == len(m1data) == len(m2data) == len(xdata), "Data length mismatch!"

    ln1.set_data(xdata, c1data)
    ln2.set_data(xdata, c2data)
    ln3.set_data(xdata, m1data)
    ln4.set_data(xdata, m2data)

    # Save to CSV
    with open(CSV_FILENAME, 'a', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([encoder, ticks])

    return ln1, ln2, ln3, ln4

ani = animation.FuncAnimation(fig, update, frames=None, init_func=init, blit=True, interval=100, save_count=10000)
plt.show()
