import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV file
data = pd.read_csv('Results_converted.csv')

# Assign the columns to variables as per your labeling
dx = data['xd']
dy = data['yd']
dz = data['zd']
x = data['x']
y = data['y']
z = data['z']
phi = data['phi']
theta = data['the']
psi = data['psi']

# 1. 3D plot for trajectory comparison
fig1 = plt.figure(figsize=(20, 16))
ax1 = fig1.add_subplot(111, projection='3d')

# Plot desired trajectory (dx, dy, dz) with dashed lines
ax1.plot(dx, dy, dz, label='Desired Trajectory', color='blue', linestyle='--', linewidth=3)

# Plot current trajectory (x, y, z) with solid lines
ax1.plot(x, y, z, label='Current Trajectory', color='red', linewidth=2, marker='o', markersize=3)

# Labels and title for the 3D plot
ax1.set_xlabel('X Position')
ax1.set_ylabel('Y Position')
ax1.set_zlabel('Z Position')
ax1.set_title('3D Trajectory Comparison')

# Add legend
ax1.legend()

# 2. 2D comparison of x, y, z with dx, dy, dz
fig2, ax2 = plt.subplots(3, 1, figsize=(20, 16))

# Plot x and dx
ax2[0].plot(x, label='Current X', color='red', linewidth=3)
ax2[0].plot(dx, label='Desired X', color='blue', linestyle='--', linewidth=3)
ax2[0].set_title('X Position Comparison')
ax2[0].set_ylabel('X Position')
ax2[0].legend()

# Plot y and dy
ax2[1].plot(y, label='Current Y', color='green', linewidth=3)
ax2[1].plot(dy, label='Desired Y', color='blue', linestyle='--', linewidth=3)
ax2[1].set_title('Y Position Comparison')
ax2[1].set_ylabel('Y Position')
ax2[1].legend()

# Plot z and dz
ax2[2].plot(z, label='Current Z', color='purple', linewidth=3)
ax2[2].plot(dz, label='Desired Z', color='blue', linestyle='--', linewidth=3)
ax2[2].set_title('Z Position Comparison')
ax2[2].set_ylabel('Z Position')
ax2[2].set_xlabel('Index')
ax2[2].legend()

# 3. Single plot comparison of phi, theta, and psi
fig3 = plt.figure(figsize=(20, 16))
ax3 = fig3.add_subplot(111)

# Plot phi
ax3.plot(phi, label='Roll', color='red', linewidth=3)

# Plot theta
ax3.plot(theta, label='Pitch', color='green', linewidth=3)

# Plot psi
ax3.plot(psi, label='Yaw', color='blue', linewidth=3)

# Labels and title for the angles comparison
ax3.set_xlabel('Index')
ax3.set_ylabel('Angle (radians)')
ax3.set_title('Roll, Pitch, and Yaw')

# Add legend
ax3.legend()

# Show all plots
plt.show()
