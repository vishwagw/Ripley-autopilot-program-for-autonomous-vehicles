from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import exception
import argparse

#connecting the drone to application through the IP address of drone.
def connectDrone():
    parser = argparse.ArgumentParser(description='Commands')
    parser.add_argument(" --connect")
    args = parser.parse_args()

    connection_string = args.connect

    vehicle = connect(connection_string, wait_ready=True)

    return vehicle

#after connecting the drone, now we are going to create the first movement which is starting the propellers and take-off.
#the take-off alsways should be an specific & selected altitude in an autonomous drone program.(use SI units always.)

def arm_and_takeoff(aTargetAltitude):
    #when vehicle in the ground:
    while not vehicle.is_armable:
        print("waiting the vehicle to become armable...")
        time.sleep(1)
    
    #for vehicle-flight mode to change to the guided mode which we are creating in ths coed.
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)
    
    #to start the vehicle
    vehicle.armed =True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed...")
        time.sleep(1)

    #take-off
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print('Current Altitude is: ' + vehicle.Location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95 :
            break
        time.sleep(1)

    #now drone has reached the target altitude.
    print("Target altitude reached...")
    return None

# now we have to activate the mission function.

#connect above functions to vehicle.(note: if previously there was an error regarding the vahicle variable, it was due to not connecting the vehicle to the function above.)
vehicle = connectDrone()
print("About to Take-Off...")

#change the mode
vehicle.mode = VehicleMode("GUIDED")
#call carm_and_takeoff function. 2 means- 2 meters above the ground(always in SI unit).
arm_and_takeoff(2)
#land back on the ground.
vehicle.mode = VehicleMode("LAND")

time.sleep(2)

print("End of functions")
print("Drone version: " + vehicle.version)

while True:
    time.sleep(2)

#closing the vehicle performance.
vehicle.close()



