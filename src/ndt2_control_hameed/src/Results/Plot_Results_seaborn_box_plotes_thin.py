import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# Load the CSV file
data = pd.read_csv('Results_converted.csv')

# Strip any whitespace characters from the column names
data.columns = data.columns.str.strip()

# Assign the columns to variables
xd = data['xd']
yd = data['yd']
zd = data['zd']
x = data['x']
y = data['y']
z = data['z']
phi = data['phi']
theta = data['the']
psi = data['psi']


# Calculate the error for each axis
error_x = np.abs(xd - x)
error_y = np.abs(yd - y)
error_z = np.abs(zd - z)

# Total position error (Euclidean distance between desired and current positions)
total_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)

# Create a 3D plot for trajectory comparison
fig1 = plt.figure(figsize=(20, 20))
ax1 = fig1.add_subplot(111, projection='3d')

# Plot the desired trajectory with increased line thickness and dot-dot style
ax1.plot(xd, yd, zd, label='Desired Ciccular Trajectory', color='red', linewidth=3, marker='o', markersize=3)

# Plot the aerial manipulator's current trajectory with increased line thickness
ax1.plot(x, y, z, label='Measured Ciccular Trajectory', color='black', linestyle='--', linewidth=3)

# Labels and title
ax1.set_xlabel('X Position', fontsize=16, fontweight='bold')
ax1.set_ylabel('Y Position', fontsize=16, fontweight='bold')
ax1.set_zlabel('Z Position', fontsize=16, fontweight='bold')
ax1.set_title('Ciccular Trajectory Tracking during Free Flight', fontsize=16, fontweight='bold')

# Set tick labels' size and bold using the fontproperties
ax1.tick_params(axis='x', labelsize=14)
ax1.tick_params(axis='y', labelsize=14)
ax1.tick_params(axis='z', labelsize=14)

# Add legend
ax1.legend()

# Create a second figure for the boxplot of errors
fig2, ax2 = plt.subplots(figsize=(12, 8))

# Prepare the data for boxplot
errors_df = pd.DataFrame({
    'Error X': error_x,
    'Error Y': error_y,
    'Error Z': error_z
})

# Create the boxplot
sns.boxplot(data=errors_df, ax=ax2)


# Add title and labels
ax2.set_title('Errors Boxplot of Ciccular Trajectory Tracking', fontsize=14, fontweight='bold')
ax2.set_ylabel('Error (meters)', fontsize=16, fontweight='bold')
ax2.set_xlabel('Axes', fontsize=16, fontweight='bold')

# Set tick labels' size and bold for boxplot
ax2.tick_params(axis='x', labelsize=14)
ax2.tick_params(axis='y', labelsize=14)

# Create a third figure for the 2D plots
fig3, axs = plt.subplots(3, 1, figsize=(10, 12))

# Plot xd vs x
axs[0].plot(xd, label='Desired X', color='red', linewidth=3, marker='o', markersize=3)
axs[0].plot(x, label='Current X', color='black', linestyle='--', linewidth=3)
axs[0].set_title('X Position Comparison', fontsize=16, fontweight='bold')
axs[0].set_ylabel('X Position (m)', fontsize=16, fontweight='bold')
axs[0].legend()
axs[0].tick_params(axis='x', labelsize=14)
axs[0].tick_params(axis='y', labelsize=14)

# Plot yd vs y
axs[1].plot(yd, label='Desired Y', color='red', linewidth=3, marker='o', markersize=3)
axs[1].plot(y, label='Current Y', color='black', linestyle='--', linewidth=3)
axs[1].set_title('Y Position Comparison', fontsize=16, fontweight='bold')
axs[1].set_ylabel('Y Position (m)', fontsize=16, fontweight='bold')
axs[1].legend()
axs[1].tick_params(axis='x', labelsize=14)
axs[1].tick_params(axis='y', labelsize=14)

# Plot zd vs z
axs[2].plot(zd, label='Desired Z', color='red', linewidth=3, marker='o', markersize=3)
axs[2].plot(z, label='Current Z', color='black', linestyle='--', linewidth=3)
axs[2].set_title('Z Position Comparison', fontsize=16, fontweight='bold')
axs[2].set_xlabel('Time Step', fontsize=16, fontweight='bold')
axs[2].set_ylabel('Z Position (m)', fontsize=16, fontweight='bold')
axs[2].legend()
axs[2].tick_params(axis='x', labelsize=14)
axs[2].tick_params(axis='y', labelsize=14)



# Adjust the layout
plt.tight_layout()

# Show all figures
plt.show()
