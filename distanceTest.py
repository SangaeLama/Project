import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import math


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    dist = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    print ("The distance is" dist)
    return dist

home = 27.608321,85.3322697
VajraSchool = 27.6064423,85.3343279

curr_location = LocationGlobalRelative(27.608321,85.3322697,20)
a_location = LocationGlobalRelative(27.6064423,85.3343279, 20)

for i in range(100):
	get_distance_metres(curr_location, a_location)	


