import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

def process_log_file(log_file_path):
    df = pd.read_csv(
        log_file_path, 
        sep=r"\] \[basic_logger\] \[info\] ", 
        engine='python', 
        names=['Timestamp', 'Data'],
        parse_dates=False
    )

    # Remove square brackets and any trailing spaces
    df['Timestamp'] = df['Timestamp'].str.strip('[] ').str.strip()

    # Use a flexible approach for datetime parsing
    def flexible_datetime_parser(x):
        try:
            return pd.to_datetime(x, format='%Y-%m-%d %H:%M:%S.%f', errors='coerce')
        except ValueError:
            return pd.NaT

    df['Timestamp'] = df['Timestamp'].apply(flexible_datetime_parser)

    # Handle any rows with NaT in Timestamp
    df.dropna(subset=['Timestamp'], inplace=True)

    df[['Parameter', 'Value']] = df['Data'].str.extract(r'(.+): (.+)')
    df['Value'] = pd.to_numeric(df['Value'], errors='coerce')

    df_pivot = df.pivot(index='Timestamp', columns='Parameter', values='Value')

    if 'Command' in df_pivot.columns:
        delta_position = df_pivot['Command'].diff()
        time_intervals = df_pivot.index.to_series().diff().dt.total_seconds()
        df_pivot['Velocity'] = delta_position / time_intervals
    return df_pivot

def plot_rejected_regions(ax, df_pivot):
    if 'Message Rejected' in df_pivot.columns:
        rejected_regions = df_pivot['Message Rejected'] == 1
        start = None
        for i, rejected in enumerate(rejected_regions):
            if rejected and start is None:
                start = df_pivot.index[i]
            elif not rejected and start is not None:
                ax.axvspan(start, df_pivot.index[i], color='red', alpha=0.3)
                start = None
        if start is not None:
            ax.axvspan(start, df_pivot.index[-1], color='red', alpha=0.3)


def calculate_moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# Paths to the log files
log_file_path_right = '../../../../logright.txt'
log_file_path_left = '../../../../logleft.txt'

# Process both log files
df_pivot_right = process_log_file(log_file_path_right)
df_pivot_left = process_log_file(log_file_path_left)

# Plotting
plt.figure(figsize=(12, 8))

# Determine the number of subplots
num_columns = max(len(df_pivot_right.columns), len(df_pivot_left.columns))
window_size = 1

for i in range(num_columns):
    ax = plt.subplot(num_columns, 1, i + 1)

    if i < len(df_pivot_right.columns):
        column = df_pivot_right.columns[i]
        ax.plot(df_pivot_right.index.to_numpy(), df_pivot_right[column].to_numpy(), label=f'Right {column}')
        plot_rejected_regions(ax, df_pivot_right)

        # if column == 'Present Load':
        #     # Calculate the reverse engineered Present Load
        #     commanded_position = df_pivot_left['Command'].to_numpy()
        #     current_position = df_pivot_left['Current Position'].to_numpy()
        #     load_difference = (np.roll(commanded_position, -1)[:-1] - current_position[:-1])*1000/3
        #     reversed_load = calculate_moving_average(load_difference, window_size)

        #     # Adjust the x-axis data to match the length of the reversed_load
        #     x_data_adjusted = df_pivot_left.index.to_numpy()[window_size-1:len(reversed_load)+window_size-1]
        #     ax.plot(x_data_adjusted, reversed_load, label='Left Reverse Engineered Load', color='orangered', linestyle='--')
    
    if i < len(df_pivot_left.columns):
        column = df_pivot_left.columns[i]
        ax.plot(df_pivot_left.index.to_numpy(), df_pivot_left[column].to_numpy(), label=f'Left {column}')
        plot_rejected_regions(ax, df_pivot_left)

        # if column == 'Present Load':
        #     # Calculate the reverse engineered Present Load
        #     commanded_position = df_pivot_right['Command'].to_numpy()
        #     current_position = df_pivot_right['Current Position'].to_numpy()
        #     load_difference = (np.roll(commanded_position, -1)[:-1] - current_position[:-1])*1000/3
        #     reversed_load = calculate_moving_average(load_difference, window_size)

        #     # Adjust the x-axis data to match the length of the reversed_load
        #     x_data_adjusted = df_pivot_right.index.to_numpy()[window_size-1:len(reversed_load)+window_size-1]
        #     ax.plot(x_data_adjusted, reversed_load, label='Right Reverse Engineered Load', color='cornflowerblue', linestyle='--')
    
    
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.set_title(f'{column} Over Time')
    ax.legend()
    ax.grid(True)

# Reduce margins and adjust layout
plt.subplots_adjust(hspace=0.4, wspace=0.3)
plt.tight_layout()
plt.show()
