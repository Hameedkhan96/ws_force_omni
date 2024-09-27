#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from inputimeout import inputimeout
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

# Rectangular trajectory parameters (user-defined)
Y_center = 0
Z_center = 1.5

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

# Main Loop
while not rospy.is_shutdown():
    if auxFlag == 0:
        mode = int(input("Mode:\n1. Position\n2. Force\n3. Slide on Rectangular trajectory (Y-Z axis)\n4. Quit\n"))
    
    if mode == 1:
        # Position mode: Set X, Y, Z position for free flight
        print("Desired position\n")
        xd = float(input("x = \n"))
        yd = float(input("y = \n"))
        zd = float(input("z = \n"))
        take_off = 1
        sw = 0
        array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, take_off, sw, 0]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        print("---------")

    elif mode == 2:
        # Force control mode
        try:
            Fd = float(inputimeout(prompt="x = \n", timeout=1))
            if Fd < 0:
                auxFlag = 0
        except Exception:
            e = Fd - F
            ie = e + e0
            xd = xd + 0.002 * e + 0.0008 * ie
            save(Fd, F, e, xd)
            auxFlag = 1
            e0 = e

        take_off = 1
        sw = 0
        array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, take_off, sw, uF]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        save(Fd, F, e, xd)
        print("---------")

    elif mode == 3:
        # Mode 3: Slide on rectangular trajectory in Y-Z plane
        print("Sliding on rectangular trajectory (Y-Z axis)...")
        width = float(input("Enter the width of the rectangle (Y axis): "))
        height = float(input("Enter the height of the rectangle (Z axis): "))
        
        # Timing adjustments for slower motion
        total_time_per_side = 80  # Increase time per side to significantly slow down the movement
        num_points_per_side = 1000  # Increase the number of interpolation points per side
        time_step = total_time_per_side / num_points_per_side
        
        # Rectangular trajectory logic
        y_positions = [-width / 2, width / 2, width / 2, -width / 2]
        z_positions = [Z_center - height / 2, Z_center - height / 2, Z_center + height / 2, Z_center + height / 2]
        
        for i in range(4):  # Loop over the four sides of the rectangle
            start_y = y_positions[i]
            end_y = y_positions[(i + 1) % 4]
            start_z = z_positions[i]
            end_z = z_positions[(i + 1) % 4]
            
            for j in range(num_points_per_side):
                # Linear interpolation between the start and end points of the current side
                alpha = j / num_points_per_side
                yd = (1 - alpha) * start_y + alpha * end_y
                zd = (1 - alpha) * start_z + alpha * end_z
                
                # Update the message array
                array = [0, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
                msg = Float32MultiArray(data=array)
                pub.publish(msg)

                print(f"Time: {j * time_step:.2f}s, Y-position: {yd:.4f}, Z-position: {zd:.4f}")
                
                time.sleep(time_step)  # Delay for real-time execution
        
        print("Rectangular trajectory complete.")
    
    elif mode == 4:
        # Quit the program
        break

    rate.sleep()
