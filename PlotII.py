import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Specify the path to your CSV file
file_path = 'InputFiles/Filtered_EM.dat'

# Read the CSV file into a DataFrame
data = pd.read_csv(file_path, header=None)

# Create a time array based on the index and assuming a sampling rate of 0.025 seconds
time = np.arange(0, len(data) * 0.025, 0.025)

# Extract the x, y, z columns of the filtered estimates
x_filtered = data.iloc[:, 0]
y_filtered = data.iloc[:, 1]
z_filtered = data.iloc[:, 2]

# Extract the x, y, z columns of the raw data
x_unfiltered = data.iloc[:, 3]
y_unfiltered = data.iloc[:, 4]
z_unfiltered = data.iloc[:, 5]

# Enable LaTeX fonts
plt.rc('text', usetex=True)

# Create subplots
fig, axs = plt.subplots(3, 1, figsize=(8, 12))

# Plot x values
axs[0].plot(time, x_filtered, label=r'Filtered')
axs[0].scatter(time, x_unfiltered, label=r'Unfiltered', color='red', marker='.', s=15)
axs[0].set_ylabel(r'$x$ [mm]', fontsize=16)

# Plot y values
axs[1].plot(time, y_filtered, label=r'Filtered', color='orange')
axs[1].scatter(time, y_unfiltered, label=r'Unfiltered', color='blue', marker='.')
axs[1].set_ylabel(r'$y$ [mm]', fontsize=16)

# Plot z values
axs[2].plot(time, z_filtered, label=r'Filtered', color='green')
axs[2].scatter(time, z_unfiltered, label=r'Unfiltered', color='purple', marker='.')
axs[2].set_ylabel(r'$z$ [mm]', fontsize=16)

# Add labels and legend
for ax in axs:
    ax.legend()
    ax.set_xlabel(r'Time [s]', fontsize=14)
    ax.minorticks_on()
    ax.grid(which='both', linestyle=':', linewidth='0.5', color='lightgrey')

# Adjust layout to prevent clipping of titles
plt.tight_layout()

# Show the plot
plt.show()