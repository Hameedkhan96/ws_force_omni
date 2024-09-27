#!/usr/bin/env python3
import time  # To introduce delay for each time step

y0 = 0   # Initial position of the aerial manipulator (Y-axis)
yf = 1   # Ask user for the final point
tf = 10  # Trajectory duration (time to complete the trajectory)
t = 0    # Start time
yd = 0   #initial valute is zero
update_time = 1 # Time interval for updating the trajectory

        # Ask user for the final position if dynamic input is required
yf = float(input("Enter the final position along the Y-axis (yf): "))

#rate = rospy.Rate(10) # 10hz

          # Trajectory update loop
while(t<=tf):
     # Cubic trajectory equation for position along Y-axis (Yd)
    yd = (6/pow(tf,2))*t*(yf-y0) - (6/pow(tf,3))*t*t*(yf-y0)  

     # Print or log the current position for visualization/debugging
    print(f"Time: {t:.2f}s, Y-position: {yd:.4f}")

    # Update the time interval
    t += update_time   

     # Introduce a delay to simulate the update rate (1 second in this case)
    time.sleep(update_time)

    
    # Update the starting position (y0) to the current position (yd) after each cycle
    y0 = yd

    # After trajectory completes, display the final position
    print(f"Final Y-position after the trajectory: {y0:.4f}")

	#rate.sleep()

    ##/*****************************************************************
    # TODO: delay (ROS rate for example), it affects update_time
    #array = [xd, yd, zd, ... ]
    #y0 =yd #update the y0 to yd outside the loop at the at of the complete cycle