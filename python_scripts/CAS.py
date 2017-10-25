from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
import math
import copy

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
parser.add_argument('--baud', default='921600')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
print 'Baud Rate: %s' % args.baud
vehicle = connect(args.connect, args.baud, wait_ready=True)
coordinate_list = []


def take_off(aTargetAltitude):
	print "Basic pre-arm checks"
	while not vehicle.is_armable:
		print "Waiting for GPS...:", vehicle.gps_0.fix_type
		print "Waiting for prechecks"
		time.sleep(1)
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	#take off to the default height
	vehicle.simple_takeoff(aTargetAltitude)
	while vehicle.location.global_relative_frame.alt <= aTargetAltitude*0.95:
		print(" Altitudeis: ", vehicle.location.global_relative_frame.alt)
		print "Reaching target altitude" 
		print " Altitude: ", vehicle.location.global_relative_frame.alt
		print " Waiting for arming..."
		print "Autopilot Firmware version: %s" % vehicle.version
		print "Global Location: %s" % vehicle.location.global_frame
		print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
		print "Local Location: %s" % vehicle.location.local_frame    #NED
		print "Attitude: %s" % vehicle.attitude
		print "Velocity: %s" % vehicle.velocity
		print "GPS: %s" % vehicle.gps_0
		print "Groundspeed: %s" % vehicle.groundspeed
		print "Airspeed: %s" % vehicle.airspeed
		print "Gimbal status: %s" % vehicle.gimbal
		print "Battery: %s" % vehicle.battery
		print "EKF OK?: %s" % vehicle.ekf_ok
		print "Last Heartbeat: %s" % vehicle.last_heartbeat
		print "Rangefinder: %s" % vehicle.rangefinder
		print "Rangefinder distance: %s" % vehicle.rangefinder.distance
		print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
		print "Heading: %s" % vehicle.heading
		print "Is Armable?: %s" % vehicle.is_armable
		print "System status: %s" % vehicle.system_status.state
		print "Mode: %s" % vehicle.mode.name    # settable
		print "Armed: %s" % vehicle.armed    # settableprintMSG
		print ""
		time.sleep(1) 

	print "Reached Altitude!"
	time.sleep(10)


def loadWayPoints():
	wayPoint = [(49.2611043, -123.2481587, 30),
				(49.2619095, -123.2487595, 30),
				(49.2620145, -123.2485235, 30),
				(49.2612163, -123.2478368, 30),
				(49.2612793, -123.2476437, 30),
				(49.2620915, -123.2483089, 30),
				(49.2621826, -123.2481050, 30),
				(49.2613844, -123.2474399, 30),
				(49.2614194, -123.2472253, 30),
				(49.2622666, -123.2478690, 30),
				(49.2621196, -123.2477188, 30)]

	for coordinate in wayPoint:
		coordinate_list.append(LocationGlobalRelative(coordinate[0], coordinate[1], coordinate[2]))

def guidedToWaypoints(point):
	poitionDiff = calculateGlobalDistance(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, point.lat, point.lon)
	while poitionDiff >= 10:
		poitionDiff = calculateGlobalDistance(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, point.lat, point.lon)
		vehicle.simple_goto(point, groundspeed = 10)
		time.sleep(5)
		print("Distance from Position: %s %s is: %s" %(vehicle.location.global_frame.lat, vehicle.global_frame.lon, poitionDiff))
	print("Reached Position %s %s" %(point.lat,point.lon))

def calculateGlobalDistance(lat1, lon1, lat2, lon2):
	R = 6378.137 #earth raiud
	dLat = lat2 * math.pi / 180 - lat1 * math.pi/180
	dLon = lon2 * math.pi / 180 - lon1 * math.pi/180
	a = math.sin(dLat/2) * math.sin(dLat/2) +math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	d = R * c;
	return d * 1000; #meters



loadWayPoints()
take_off(30)
for coordinate in coordinate_list:
	guidedToWaypoints(coordinate)

print "end of script"
vehicle.close()