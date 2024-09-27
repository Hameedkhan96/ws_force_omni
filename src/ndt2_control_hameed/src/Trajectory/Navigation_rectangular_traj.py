#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped
import time

# Global variables
global F, Fd, e, ie, e0, uF
F = 0
Fd = 0.5
e = 0
ie = 0
e0 = 0
uF = 0

# Variables to store the last position from Mode 1 (x, y, z)
last_x = 0
last_y = 0
last_z = 0

def force_callback(data):
    global F
    F = data.wrench.force.x  # Extract the force data from sensor

def save(Fd, F, e, xd):
    with open('Force.txt', 'a') as file:
        file.write(f'{Fd},{F},{e},{xd}\n')

# ROS Initialization
rospy.init_node('Navigation', anonymous=True)

# Publishers and Subscribers
pub = rospy.Publisher('references', Float32MultiArray, queue_size=10)
rospy.Subscriber("/wrench_estimation", WrenchStamped, force_callback)
rate = rospy.Rate(10)

auxFlag = 0
first_run = True

# Function to return to the starting position
def return_to_start(start_pos, pub):
    print("Returning to starting position...")
    total_time = 3  # Faster return to starting position
    num_steps = 30  # Faster steps to starting position
    time_step = total_time / num_steps

    current_pos = [last_x, last_y, last_z]
    for i in range(num_steps):
        alpha = i / num_steps
        yd = (1 - alpha) * current_pos[1] + alpha * start_pos[1]
        zd = (1 - alpha) * current_pos[2] + alpha * start_pos[2]
        
        array = [last_x, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        time.sleep(time_step)

# Main Loop
while not rospy.is_shutdown():
    if auxFlag == 0:
        mode = int(input("Mode:\n1. Position\n2. Force\n3. Slide on Rectangular trajectory (Z-Y axis)\n4. Quit\n"))
    
    if mode == 1:
        # Position mode: Set X, Y, Z position for free flight
        print("Desired position\n")
        last_x = float(input("x = \n"))  # Store last X position
        last_y = float(input("y = \n"))  # Store last Y position
        last_z = float(input("z = \n"))  # Store last Z position
        take_off = 1
        sw = 0
        array = [last_x, last_y, last_z, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, take_off, sw, 0]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        print("---------")

    elif mode == 3:
        # Mode 3: Slide on rectangular trajectory in Z-Y plane (starting with Z)
        print("Sliding on rectangular trajectory (Z-Y axis)...")

        width = float(input("Enter the width of the rectangle (Y axis): "))
        height = float(input("Enter the height of the rectangle (Z axis): "))

        # Adjust for the total trajectory duration of 50 seconds
        total_time = 50  # Total trajectory time (50 seconds)
        pause_time_Y_to_Z = 4  # 4-second pause at Y to Z transition
        pause_time_Z_to_Y = 2  # 2-second pause at Z to Y transition
        moving_time = total_time - (2 * 2 + 2 * 4)  # Subtracting pause times
        time_per_side = moving_time / 4  # Time per side of the rectangle

        num_points_per_side = 100  # Number of points for smoother motion
        time_step = time_per_side / num_points_per_side

        # Step 1: Start by ascending along Z axis (Z = 0 to Z = height)
        start_z = last_z
        end_z = last_z + height

        print("Starting immediate ascending along Z axis...")
        for j in range(num_points_per_side):
            alpha = j / num_points_per_side
            zd = (1 - alpha) * start_z + alpha * end_z
            yd = last_y  # Y stays constant during Z ascent
            print(f"Time: {j*time_step:.2f}s, Y-position: {yd:.4f}, Z-position: {zd:.4f}")
            array = [last_x, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            time.sleep(time_step)

        # Pause at the top of Z axis for 2 seconds
        print("Pausing for 2 seconds at the Z max corner...")
        time.sleep(pause_time_Z_to_Y)

        # Step 2: Slide along Y axis (Y = 0 to Y = width)
        start_y = last_y
        end_y = last_y + width
        zd = end_z  # Z remains constant at full height during Y slide
        print("Starting immediate sliding along Y axis...")

        for j in range(num_points_per_side):
            alpha = j / num_points_per_side
            yd = (1 - alpha) * start_y + alpha * end_y
            print(f"Time: {(j + num_points_per_side)*time_step:.2f}s, Y-position: {yd:.4f}, Z-position: {zd:.4f}")
            array = [last_x, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            time.sleep(time_step / 1.5)  # Faster sliding along Y axis

        # Pause at the corner for 4 seconds before descending
        print("Pausing for 4 seconds at the Y max corner...")
        time.sleep(pause_time_Y_to_Z)

        # Step 3: Descend along Z axis (Z = height back to 0)
        start_z = end_z
        end_z = last_z  # Descend back to starting Z position
        yd = end_y  # Y remains constant during descent
        for j in range(num_points_per_side):
            alpha = j / num_points_per_side
            zd = (1 - alpha) * start_z + alpha * end_z
            print(f"Time: {(j + 2*num_points_per_side)*time_step:.2f}s, Y-position: {yd:.4f}, Z-position: {zd:.4f}")
            array = [last_x, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            time.sleep(time_step)

        # Pause at the bottom of Z axis for 2 seconds
        print("Pausing for 2 seconds at the Z min corner...")
        time.sleep(pause_time_Z_to_Y)

        # Step 4: Slide back along Y axis (Y = width back to 0)
        start_y = end_y
        end_y = last_y  # Slide back to original Y position
        zd = end_z  # Z remains constant at starting height (after descent)
        print("Starting immediate sliding back along Y axis...")
        for j in range(num_points_per_side):
            alpha = j / num_points_per_side
            yd = (1 - alpha) * start_y + alpha * end_y
            print(f"Time: {(j + 3*num_points_per_side)*time_step:.2f}s, Y-position: {yd:.4f}, Z-position: {zd:.4f}")
            array = [last_x, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            time.sleep(time_step / 1.5)  # Faster sliding along Y axis

        print("Rectangular trajectory complete.")

        # Return to starting position
        return_to_start([last_x, last_y, last_z], pub)

    elif mode == 4:
        # Quit the program
        break

    rate.sleep()
