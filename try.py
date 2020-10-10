#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""


from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from math import sin, cos, sqrt, atan2, radians
import numpy as np
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

""" 
Set Wind Direction to 180 
and Wind Speed to 10
"""
 
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(20)

    
def get_distance_metres(aLocation1, aLocation2):
	#Applying logic from a research paper found on the net
	# approximate radius of earth in km
	R = 6373.0
	lat1 = radians(aLocation1.lat)
	lat2 = radians(aLocation2.lat)
	dlat = radians(aLocation2.lat - aLocation1.lat)
	dlong = radians(aLocation2.lon - aLocation1.lon)
	a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlong / 2)**2
	#c=2*np.arcsin(sqrt(a))
	c = 2 * atan2(sqrt(a), sqrt(1 - a))
	distance = R * c
	return distance

def PAYLOADDROP():
	h=20
	v=5
	g=9.8
	#assuming wind speed to be 0
	m=2	#kg
	area=4*3.14*2*2 #surface area = 4pi*r^2
	airdensity=1.204 #kg/m**3 at 20 degrees by google
	q=0.5*airdensity*area*0.5 # by google drag coefficient of sphere in air is 0.5
	R = 6373.0
	t=0.02
	N=3000
	i=0		
	vy=0		#vertical velocity
	vx=v		#horizontal velocity
	x=0
	y=0
	tf=0
	target= LocationGlobalRelative(-35.36248343, 149.16512541)
	current=vehicle.location.global_frame
	while i<N:
		ax=-(q/m)*v**2
		ay=g
		vx=vx+ax*t
		vy=vy+ay*t
		xf=x+vx*t+0.5*ax*t**2
		yf=y+vy*t+0.5*ay*t**2
		x=xf
		y=yf
		tf=tf+t
		i=i+1
		if(y==h):
			range=x
			break
	angle=np.arctan(radians((target.lon-current.lon)/(target.lat-target.lon)))
	RPlat=radians(target.lat)-R*sin(radians(angle))
	RPlong=radians(target.lon)-R*cos(radians(angle))
	
	drop=LocationGlobalRelative(RPlat,RPlong)
	print("Payload should be dropped at %s",drop)
	return drop
	
#SCRIPT 1
def UAVNAVIGATION():
	print("Set default/target airspeed to 5")
	vehicle.airspeed = 5
	home= vehicle.location.global_frame
	print(" Global Location: %s" % vehicle.location.global_frame)
	time.sleep(3)
	#	The Waypoint Coordinates
	point1 = LocationGlobalRelative(-35.364260, 149.164037, 20.000000)
	point2= LocationGlobalRelative(-35.362609, 149.163528, 20.000000)
	point3= LocationGlobalRelative(-35.362452, 149.166185, 20.000000)
	point4= LocationGlobalRelative(-35.363811, 149.166405, 20.000000)

	#towards waypoint 1

	print("Going towards first point")
	vehicle.simple_goto(point1)
	while True:
		distancetopoint = get_distance_metres(vehicle.location.global_frame, point1)
		print(" Global Location: %s \n Distance from waypoint 1 %s" %(vehicle.location.global_frame,distancetopoint))
		if distancetopoint<0.001:
			print("Reached Waypoint1")
			time.sleep(3)
			break
            
	#Towards Waypoint 2
	print("Going towards Second point ")
	vehicle.simple_goto(point2)
	while True:
		distancetopoint = get_distance_metres(vehicle.location.global_frame, point2)
		print(" Global Location: %s \n Distance from waypoint 2 %s" %(vehicle.location.global_frame,distancetopoint))
		if distancetopoint<0.001:
			print("Reached Waypoint2")
			time.sleep(3)
			break

	#Towards Waypoint 3

	print("Going towards Third point ")
	vehicle.simple_goto(point3)
	while True:
		distancetopoint = get_distance_metres(vehicle.location.global_frame, point3)
		print(" Global Location: %s \n Distance from waypoint 3 %s" %(vehicle.location.global_frame,distancetopoint))
		if distancetopoint<0.001:
			print("Reached Waypoint3")
			time.sleep(3)
			break;
			
	x=PAYLOADDROP()
	time.sleep(3)
	#Towards Waypoint 4
	print("Going towards Fourth point ")
	vehicle.simple_goto(point4)
	while True:
		if get_distance_metres(vehicle.location.global_frame,x)<0.001:
			print("PAYLOAD DROPPED SUCCESSFULLY at %s" %x)
			sleep(2)
		distancetopoint = get_distance_metres(vehicle.location.global_frame, point4)
		print(" Global Location: %s \n Distance from waypoint 4 %s" %(vehicle.location.global_frame,distancetopoint))
		if distancetopoint<0.001:
			print("Reached Waypoint4")
			break
	print("Returning to Launch")
	vehicle.mode = VehicleMode("RTL")
	while True:
		distancetopoint = get_distance_metres(vehicle.location.global_frame, home)
		print(" Global Location: %s \n Distance from Home %s" %(vehicle.location.global_frame,distancetopoint))
		if distancetopoint<0.001:
			print("Reached Home")
			break
	time.sleep(3)

	# Close vehicle object before exiting script
	print("Close vehicle object")
	vehicle.close()
	
#DRIVER CODE

UAVNAVIGATION()


