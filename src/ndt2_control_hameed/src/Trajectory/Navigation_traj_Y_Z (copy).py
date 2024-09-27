#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from inputimeout import inputimeout
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time  # For timing in cubic trajectory

# Global variables
global F, Fd, Fd0, e, ie, e0, uF
F = 0
Fd = 0.5
Fd0 = 0
ie = 0
e0 = 0
uF = 0
z0 = 1  # Initialize with the default start Z position

def force_callback(data):
    global F
    F = data.wrench.force.x  # Extract the force data from sensor

def save(Fd, F, e, xd):
    # Save control data to a file (adjust file path as needed)
    with open('Force.txt', 'a') as file:
        file.write(f'{Fd},{F},{e},{xd}\n')

# ROS Initialization
rospy.init_node('Navigation', anonymous=True)

# Publishers and Subscribers
pub = rospy.Publisher('references', Float32MultiArray, queue_size=10)
rospy.Subscriber("/wrench_estimation", WrenchStamped, force_callback)
rate = rospy.Rate(10)  # 10Hz rate

auxFlag = 0

# Cubic Trajectory Variables
y0 = 0  # Initial Y position
z0 = 0  # Initial Z position
update_time = 0.05  # Slower time interval for more stability

# Main Loop
while not rospy.is_shutdown():
    if auxFlag == 0:
        mode = int(input("Mode:\n1. Position\n2. Force\n3. Slide on Y or Z axis (Cubic Trajectory)\n4. Quit\n"))
    
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
        # Force control mode: Apply force to push the surface
        try:
            Fd = float(inputimeout(prompt="x = \n", timeout=1))

            if Fd < 0:
                auxFlag = 0

        except Exception:
            e = Fd - F
            ie = e + e0
            xd = xd + 0.002 * e + 0.0008 * ie
            print("u = " + str(xd) + " e = " + str(e))
            print("Fd = " + str(Fd) + ", F = " + str(F))
            auxFlag = 1
            e0 = e
        
        take_off = 1
        sw = 0
        array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, take_off, sw, uF]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        print("---------")
    
    elif mode == 3:
        # Mode 3: Slide on either Y-axis or Z-axis using cubic trajectory
        print("Choose axis for sliding trajectory:")
        axis_choice = input("Enter 'Y' for Y-axis or 'Z' for Z-axis: ").strip().upper()

        if 'xd' not in globals():
            xd = 0  # Default X position

        if axis_choice == 'Y':
            # Ask user for the final Y position
            yf = float(input("Enter the final Y position (yf): "))
            tf = abs(yf - y0) * 2  # Scale time to slow down the trajectory
            
            # Loop to execute cubic trajectory on Y axis
            t = 0  # Reset time for trajectory
            while t <= tf:
                # Cubic trajectory equation for Y axis
                yd = (6 / pow(tf, 2)) * t * (yf - y0) - (6 / pow(tf, 3)) * t * t * (yf - y0)
                
                if 'xd' not in globals():
                    zd = 0  # Default X position

                # Update array with sliding Y-axis position and lock Z-axis to initial value
                array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
                msg = Float32MultiArray(data=array)
                
                # Publish continuously at every step
                pub.publish(msg)
                
                # Print for debugging/logging
                print(f"Time: {t:.2f}s, Y-position: {yd:.4f}")
                
                # Update time for next iteration
                t += update_time
                
                # Delay to simulate real-time execution
                time.sleep(update_time)
                
                # Update initial Y position to the current one (for continuous sliding)
                y0 = yd

            # Stop the drone at the final Y position
            array = [xd, yf, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            print(f"Final Y position reached: {yf:.4f}")

        elif axis_choice == 'Z':
            # Ask user for the final Z position
            zf = float(input("Enter the final Z position (zf): "))
            # Dynamically adjust the time factor based on the distance
            distance = abs(zf - z0)
            tf = max(5, distance * 5)  # Use a minimum tf of 5 for small movements

            update_time = 0.05  # Time between updates
            t = 0

            while t <= tf:
                # Cubic trajectory equation for Z axis
                zd = z0 + (zf - z0) * (3 * pow(t/tf, 2) - 2 * pow(t/tf, 3))

                # Limit the position to avoid overshooting
                zd = min(max(zd, min(z0, zf)), max(z0, zf))

                # Update array with current Z position
                array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
                msg = Float32MultiArray(data=array)
                pub.publish(msg)

                print(f"Time: {t:.2f}s, Z-position: {zd:.4f}")
                t += update_time
                time.sleep(update_time)  # Ensure real-time movement

                # Update initial Z position to the current one (for continuous sliding)
                z0 = zd

            # Stop the drone at the final Z position
            array = [xd, yd, zf, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            print(f"Final Z position reached: {zf:.4f}")
    
    elif mode == 4:
        # Quit the program (formerly Mode 3)
        break

    rate.sleep()

