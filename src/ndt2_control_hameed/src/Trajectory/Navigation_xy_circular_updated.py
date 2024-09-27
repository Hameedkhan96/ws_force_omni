#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from inputimeout import inputimeout
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time  # For timing in cubic trajectory

# Global variables
global F, Fd, e, ie, e0, uF
F = 0
Fd = 0.5
e = 0
ie = 0
e0 = 0
uF = 0

# Circular trajectory parameters
X_center = 0
Y_center = 0
radius = 1.0
Z_constant = 1.5  # Z axis will remain constant

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

# Main Loop
while not rospy.is_shutdown():
    if auxFlag == 0:
        mode = int(input("Mode:\n1. Position\n2. Force\n3. Circular trajectory on X and Y axis\n4. Quit\n"))

    if mode == 1:
        # Free flight position control mode
        print("Desired position")
        xd = float(input("x = "))
        yd = float(input("y = "))
        zd = float(input("z = "))
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
            print(f"u = {xd} e = {e}")
            print(f"Fd = {Fd}, F = {F}")
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
        # Circular trajectory on X-Y axis
        print("Sliding on circular trajectory (X-Y axis)...")

        # Circular parameters
        theta = 0  # Starting angle in radians
        max_theta = 2 * np.pi  # Full circle

        # Time control parameters
        tf = 120  # Total time for a full circle
        update_time = 0.05  # Time between updates
        t = 0

        # Define the constant Z position
        Z_constant = 1.5  # Adjust this value as needed for constant Z

        while t <= tf:
            # Parametric equations for a circular trajectory
            theta = (t / tf) * max_theta  # Map time t to angle theta (0 to 2*pi)

            # Calculate X and Y positions
            xd = X_center + radius * np.cos(theta)  # X-axis circular motion
            yd = Y_center + radius * np.sin(theta)  # Y-axis circular motion
            zd = Z_constant  # Z-axis remains constant

            # Update array with current X, Y, and constant Z positions
            array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)

            print(f"Time: {t:.2f}s, X-position: {xd:.4f}, Y-position: {yd:.4f}")
            t += update_time
            time.sleep(update_time)  # Ensure real-time movement

        print("Circular trajectory complete.")

    elif mode == 4:
        # Quit mode
        break

    rate.sleep()
