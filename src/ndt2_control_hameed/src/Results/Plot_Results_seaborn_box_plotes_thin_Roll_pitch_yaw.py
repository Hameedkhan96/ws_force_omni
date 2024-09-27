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
fx = data['fx']
fx_est = data['wrench_est.wrench.force.x']

# Calculate the error for each axis
error_x = np.abs(xd - x)
error_y = np.abs(yd - y)
error_z = np.abs(zd - z)

# Total position error (Euclidean distance between desired and current positions)
total_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)

# Create a 3D plot for trajectory comparison
fig1 = plt.figure(figsize=(20, 16))
ax1 = fig1.add_subplot(111, projection='3d')

# Plot the desired trajectory with increased line thickness and dot-dot style
ax1.plot(xd, yd, zd, label='Desired Trajectory', color='red', linewidth=3, marker='o', markersize=3)

# Plot the aerial manipulator's current trajectory with increased line thickness
ax1.plot(x, y, z, label='Measured Trajectory', color='black', linestyle='--', linewidth=3)

# Labels and title
ax1.set_xlabel('X Position (m)', fontsize=16, fontweight='bold')
ax1.set_ylabel('Y Position (m)', fontsize=16, fontweight='bold')
ax1.set_zlabel('Z Position (m)', fontsize=16, fontweight='bold')
ax1.set_title('Circular trajectory tracking while free flight', fontsize=16, fontweight='bold')

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
ax2.set_title('Errors Boxplot circular trajectory tracking while free flight', fontsize=14, fontweight='bold')
ax2.set_ylabel('Error (m)', fontsize=16, fontweight='bold')
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
axs[2].set_xlabel('Time (ms)', fontsize=16, fontweight='bold')
axs[2].set_ylabel('Z Position (m)', fontsize=16, fontweight='bold')
axs[2].legend()
axs[2].tick_params(axis='x', labelsize=14)
axs[2].tick_params(axis='y', labelsize=14)

# Adjust the layout
plt.tight_layout()

# Create a fourth figure for Roll (phi), Pitch (theta), and Yaw (psi)
fig4, ax4 = plt.subplots(figsize=(12, 8))

# Plot roll, pitch, and yaw
ax4.plot(phi, label='Roll', color='blue', linewidth=3, marker='o', markersize=3)
ax4.plot(theta, label='Pitch', color='black', linewidth=3, marker='o', markersize=3)
ax4.plot(psi, label='Yaw', color='red', linewidth=3, marker='o', markersize=3)

# Add title and labels
ax4.set_title('Roll, Pitch, and Yaw Comparison', fontsize=16, fontweight='bold')
ax4.set_xlabel('Time (ms)', fontsize=16, fontweight='bold')
ax4.set_ylabel('Angle (Radians)', fontsize=16, fontweight='bold')

# Set tick labels' size and bold for the 4th plot
ax4.tick_params(axis='x', labelsize=14)
ax4.tick_params(axis='y', labelsize=14)

# Add legend
ax4.legend()

# Adjust the layout
plt.tight_layout()




# Create a fifth figure for applied force
fig5, ax5 = plt.subplots(figsize=(12, 8))

# Plot roll, pitch, and yaw
ax5.plot(fx, label='Applied force', color='black', linewidth=3, marker='o', markersize=3)

# Add title and labels
ax5.set_title('Applied force', fontsize=16, fontweight='bold')
ax5.set_xlabel('Time (ms)', fontsize=16, fontweight='bold')
ax5.set_ylabel('Force (N)', fontsize=16, fontweight='bold')

# Set tick labels' size and bold for the 4th plot
ax5.tick_params(axis='x', labelsize=14)
ax5.tick_params(axis='y', labelsize=14)

# Add legend
ax5.legend()

# Adjust the layout
plt.tight_layout()


# Show all figures
plt.show()
