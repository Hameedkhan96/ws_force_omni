import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
data = pd.read_csv('Force_converted_updated.csv')

# Extract the columns
Fd = data['Fd']        # Desired force (Column 1)
F = data['F']          # Measured force (Column 2)
Error = data['Error']  # Error (Column 3)

# 1. Plot Fd and F comparison with increased font size and bold text
plt.figure(figsize=(10, 6))
plt.plot(Fd, label='Fd (Desired Force)', color='black', linestyle='--', linewidth=3)
plt.plot(F, label='F (Measured Force)', color='red', linewidth=3)

# Increase font size and make labels bold
plt.title('Comparison of Desired and Measured Force', fontsize=18, fontweight='bold')
plt.xlabel('Time (s)', fontsize=14, fontweight='bold')
plt.ylabel('Force (N)', fontsize=14, fontweight='bold')

# Customize tick parameters
plt.xticks(fontsize=14, fontweight='bold')
plt.yticks(fontsize=14, fontweight='bold')

# Add legend with larger font size
plt.legend(fontsize=14)

# Add grid for better clarity
#plt.grid(True)

# Show the plot
plt.show()

# 2. Plot the error (No changes required for this figure based on your request)
plt.figure(figsize=(10, 6))
plt.plot(Error, label='Error (Fd - F)', color='blue', linewidth=2)
plt.title('Error in Force', fontsize=18, fontweight='bold')
plt.xlabel('Time (s)', fontsize=14, fontweight='bold')
plt.ylabel('Error (N)', fontsize=14, fontweight='bold')
plt.xticks(fontsize=14, fontweight='bold')
plt.yticks(fontsize=14, fontweight='bold')
plt.legend(fontsize=14)
#plt.grid(True)
plt.show()
