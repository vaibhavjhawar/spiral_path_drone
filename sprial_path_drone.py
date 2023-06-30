#!/usr/bin/python2

# import python libraries
import sys
from time import sleep
from math import pi, sin, cos

# import drone libraries
from pymavlink import mavutil
from dronekit import connect, VehicleMode

# import OpenCV
import cv2 as cv

# try opening camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    sys.exit("Cannot open camera")

# print "Start simulator (SITL)"
# import dronekit_sitl
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()

print "\nStarting Mission\n"

# Connect to the Vehicle
connection_string = "127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=False)

# Get some vehicle attributes (state)
print "\nGet some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable

print "\n"


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        sleep(1)

# Takeoff to specific altitude and hover until next command
arm_and_takeoff(5)
sleep(3)

# Set up velocity mappings
# velocity_x > 0 => fly North
# velocity_x < 0 => fly South
# velocity_y > 0 => fly East
# velocity_y < 0 => fly West
# velocity_z < 0 => ascend
# velocity_z > 0 => descend

print "\nTravelling north"

def send_ned_velocity2(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        sleep(1)

# Travel in north direction for some time and wait for next command
send_ned_velocity2(4, 0, 0, 10)
sleep(5)


def send_ned_velocity(v_t, v_f, UP, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    n = 0.07
    for x in range(0,duration):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            v_t*sin(2*pi*n*x)+v_f, v_t*cos(2*pi*n*x)+v_f, UP, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    
        # send command to vehicle on 1 Hz cycle
        vehicle.send_mavlink(msg)
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        sleep(1)

v_t = 14
v_f = 0

UP = -0.5
DURATION = 20

print "\nStarting Spiral Path ..."

# Execute spiral path for given duration
send_ned_velocity(v_t, v_f, UP, DURATION)
sleep(3)


# Read camera frame and detect arUco marker 
print "\nwaiting for aruco marker detection"

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # Display frame on screen
    cv.imshow('video', frame)

    a_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    a_params = arucoParams = cv.aruco.DetectorParameters_create()

    # Detect arUco markers in frame
    corners, id, rejected = cv.aruco.detectMarkers(frame, a_dict, parameters=a_params)

    if id is not None:
        print "arUco id detected"
        break

    if cv.waitKey(1) == ord('q'):
        break


# Finally land the drone
print "\nLanding!"
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
# sitl.stop()
print("Mission Completed")


# When everything done, release the capture
cap.release()
cv.destroyAllWindows()

