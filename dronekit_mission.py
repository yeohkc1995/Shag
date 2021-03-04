################################################################################################
# @File DroneKitPX4.py
# Example usage of DroneKit with PX4
#
# @author Sander Smeets <sander@droneslab.com>
#
# Code partly based on DroneKit (c) Copyright 2015-2016, 3D Robotics.
################################################################################################

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math
from Haversine import Haversine_XY
import threading

################################################################################################
# Settings
################################################################################################

#connection_string       = '127.0.0.1:14540'
connection_string       = '/dev/ttyACM0'

MAV_MODE_AUTO   = 4
# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py


# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print("Connecting to vehicle...\n\n")

result = None
while result is None:
    try:
        vehicle = connect(connection_string, wait_ready=True, baud=57600)
        result = 1
    except:
            pass

print("\n\nConnected to Vehicle\n\n")


### KC: GUI thread to run concurrently
def GUI_thread():
    global command_string, offset_dist, offset_brg
    print("\n\n\nPlease input your command: \n\n\n")
    while command_string != "end":
        command_string = raw_input()
        if command_string == "E":
            offset_dist = 10
        elif command_string == "E10":
            offset_dist = 20
        elif command_string == "N1":
            offset_brg = 180
        elif command_string == "N2":
            offset_brg = 0
        elif command_string == "N3":
            offset_brg = 270
        elif command_string == "N4":
            offset_brg = 90
        else:
            print("\nWrong input la!\n")

    print("\n\n\nGUI closed!\n\n\n")

### KC: GPS thread to run concurrently
def GPS_thread():
    global zoon_lat, zoon_lon, zoon_alt, zoon_bearing, wp, target_x, target_y, target_z
    zoon_lat = 1.2997019688080589
    zoon_lon = 103.78722860926898

    while True:
        zoon_lat += 0.00001
        zoon_lon += 0.00001
        time.sleep(1)
        print("Latitude: %f,  Longitude: %f" %(zoon_lat, zoon_lon))

        #target_x, target_y, d = Haversine_XY(wp.lat, wp.lon, zoon_lat, zoon_lon)
        #print("X: %f,  Y: %f" %(target_x, target_y))

### KC: Serial thread to read instructions
def XBee_thread():
    global zoon_lat, zoon_lon, zoon_alt, zoon_bearing, offset_dist, offset_brg, zoon_vel

    zoon_lat = 1.2997019688080589
    zoon_lon = 103.78722860926898

    print("Connecting to XBee...")
    
    result = None
    while result is None:
        try:
            print("Connecting to XBee...")
            time.sleep(1)
            ser = serial.Serial('/dev/ttyUSB0')
            ser.baudrate = 9600
            result = 1
        except:
             pass

    print("\n\nConnected to XBee\n\n")    

    while True:
    ##serial code to read inputs
        x = ser.readline()
        x_dict = x.split(",")
        print("XBee Reading: ")
        print(x_dict)
        
        comd_brg = x_dict[0]
        comd_dist = x_dict[1]
        zoon_lat = float(x_dict[2])
        zoon_lon = float(x_dict[3])
        zoon_bearing = float(x_dict[4])
        temp_vel = float(x_dict[5])
        zoon_alt = float(x_dict[6])
        
   
        

        if comd_dist == "E":
            offset_dist = 10
        elif comd_dist == "E10":
            offset_dist = 20
        elif comd_brg == "N1":
            offset_brg = 180
        elif comd_brg == "N2":
            offset_brg = 0
        elif comd_brg == "N3":
            offset_brg = 270
        elif comd_brg == "N4":
            offset_brg = 90

        if temp_vel < 1:
            zoon_vel = 1
        elif temp_vel >= 1:
            zoon_vel = temp_vel

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



def get_location_offset_meters(original_lat, original_lon, original_alt, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_lat/180))

    #New position in decimal degrees
    newlat = original_lat + (dLat * 180/math.pi)
    newlon = original_lon + (dLon * 180/math.pi)
    return newlat, newlon, original_alt+alt



################################################################################################
# Listeners
################################################################################################



### KC: global variable
command_string = ""
offset_dist = 20
offset_brg = 0
zoon_lat = 0
zoon_lon = 0
zoon_alt = 20
zoon_bearing = 0
zoon_vel = 1
target_x = 0
target_y = 0
target_z = 0

vehicle.armed = True


################################################################################################
# Start mission example
################################################################################################

### KC: start threads ###
#x = threading.Thread(target=GUI_thread)
#x.start()

#y = threading.Thread(target=GPS_thread)
#y.start()

x = threading.Thread(target=XBee_thread)
x.start()


# Display basic vehicle state
print(" Type: %s" % vehicle._vehicle_type)
print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)

# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

# Arm vehicle
vehicle.armed = True

# Set mode to guided - this is optional as the goto method will change the mode if needed.
vehicle.mode = VehicleMode("GUIDED")

while True:
    # Set the target location in global-relative frame

    theta = (zoon_bearing + offset_brg) % 360
    dE = offset_dist * math.sin(math.radians(theta))
    dN = offset_dist * math.cos(math.radians(theta))

    
    X, Y, Z = get_location_offset_meters(zoon_lat, zoon_lon, zoon_alt, dN, dE, 10)

    a_location = LocationGlobalRelative(X, Y, Z) #1.2997019688080589, 103.78722860926898
    vehicle.simple_goto(a_location)
    vehicle.airspeed = zoon_vel
    print("Going to location (lat: %.6f, lon: %.6f, alt: %f)" %(X, Y, Z))
    print("Offset Dist: %f, Offset Bearing: %f" %(offset_dist, offset_brg))

    time.sleep(1)


