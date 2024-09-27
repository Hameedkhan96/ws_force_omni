#!/usr/bin/env python3

y0 = 0  #the aerial manipulator starts in 0 but requires to be updated regularly at each interval
yf = 1  #ask user for the final point yf = need to enter after the complete cycle
tf = 10  #trajectory duration
t = 0    #starting time, start at time t = 0
yd = 0   #initial valute is zero
update_time = 1 #Depends on the delay

while(t<=tf):
    yd = (6/pow(tf,2))*t*(yf-y0) - (6/pow(tf,3))*t*t*(yf-y0)  #the trajetory equation
    t = t+update_time    # update the time interval each time
   
    #TODO: delay (ROS rate for example), it affects update_time
    #array = [xd, yd, zd, ... ]
 y0 =yd #update the y0 to yd outside the loop at the at of the complete cycle