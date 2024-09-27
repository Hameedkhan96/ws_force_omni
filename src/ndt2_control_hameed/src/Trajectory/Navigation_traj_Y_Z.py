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
z0 = 1  # Initial Z position
y0 = 0  # Initial Y position

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
        mode = int(input("Mode:\n1. Position\n2. Force\n3. Slide on Y or Z-axis (Cubic Trajectory)\n4. Quit\n"))

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
        # Mode 3: Choose to slide on Y-axis or Z-axis
        print("Choose axis for sliding:")
        axis_choice = input("Enter 'Y' for Y-axis or 'Z' for Z-axis: ").strip().upper()

        if axis_choice == 'Y':
            # Ask for the final Y position
            yf = float(input("Enter the final Y position (yf): "))

            # Dynamically adjust the time factor based on the distance
            distance = abs(yf - y0)
            tf = max(5, distance * 5)  # Use a minimum tf of 5 for small movements

            update_time = 0.05  # Time between updates
            t = 0

            while t <= tf:
                # Cubic trajectory equation for Y-axis
                yd = y0 + (yf - y0) * (3 * pow(t/tf, 2) - 2 * pow(t/tf, 3))

                # Limit the position to avoid overshooting
                yd = min(max(yd, min(y0, yf)), max(y0, yf))

                # Update array with current Y position
                array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
                msg = Float32MultiArray(data=array)
                pub.publish(msg)

                print(f"Time: {t:.2f}s, Y-position: {yd:.4f}")
                t += update_time
                time.sleep(update_time)  # Ensure real-time movement

            # Update the starting Y position to the final Y position for future runs
            y0 = yf

            # Stop at final Y position
            array = [xd, yf, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            print(f"Final Y position reached: {yf:.4f}")

        elif axis_choice == 'Z':
            # Ask for the final Z position
            zf = float(input("Enter the final Z position (zf): "))

            # Dynamically adjust the time factor based on the distance
            distance = abs(zf - z0)
            tf = max(5, distance * 5)  # Use a minimum tf of 5 for small movements

            update_time = 0.05  # Time between updates
            t = 0

            while t <= tf:
                # Cubic trajectory equation for Z-axis
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

            # Update the starting Z position to the final Z position for future runs
            z0 = zf

            # Stop at final Z position
            array = [xd, yd, zf, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            print(f"Final Z position reached: {zf:.4f}")

    elif mode == 4:
        # Quit mode
        break

    rate.sleep()
