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
ax1.plot(xd, yd, zd, label='Desired Trajectory', color='blue', linestyle='--', linewidth=3)

# Plot the aerial manipulator's current trajectory with increased line thickness
ax1.plot(x, y, z, label='Current Trajectory', color='red', linewidth=2, marker='o', markersize=3)

# Labels and title
ax1.set_xlabel('X Position')
ax1.set_ylabel('Y Position')
ax1.set_zlabel('Z Position')
ax1.set_title('Trajectory Comparison')

# Add legend
ax1.legend()

# Create a second figure for the violin plot of errors
fig2, ax2 = plt.subplots(figsize=(12, 8))

# Prepare the data for violin plot
errors_df = pd.DataFrame({
    'Error X': error_x,
    'Error Y': error_y,
    'Error Z': error_z
})

# Create the violin plot
sns.violinplot(data=errors_df, ax=ax2)

# Add title and labels
ax2.set_title('Violin Plot of Trajectory Errors')
ax2.set_ylabel('Error (meters)')
ax2.set_xlabel('Axes')

# Show both plots
plt.show()
