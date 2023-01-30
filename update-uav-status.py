#!/usr/bin/env python3

import sys
import time
#import serial
from pymavlink import mavutil

# MAVLINK variables initialization
AUTOPILOT_ARDUPILOT = 3
VEHICLE_TYPE_QUAD = 2

# Device initialization
try:
	# Get the armed status from the armed status file if it exists
	f = open("/etc/default/xoss-armed-status", "r")
	status_line = f.readline()
	f.close()

	prev_armed = status_line.splitlines()[0]

	if prev_armed != "ARMED" and prev_armed != "DISARMED":
		raise ValueError
except (FileNotFoundError, IndexError, ValueError):
	# Armed status file does not exist or its contents is not valid - create it with valid contents
	prev_armed="DISARMED"
	f = open("/etc/default/xoss-armed-status", "w")
	f.write(prev_armed+"\n")
	f.close()

local_port = sys.argv[1]
device_baudrate = int(sys.argv[2])
device = 'udpin:localhost:'+local_port # Create a connection listening to a local UDP port
vehicle = mavutil.mavlink_connection(device,baud=device_baudrate)
vehicle.wait_heartbeat()

while True:
	# Wait for heartbeat from the flight controller
	msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
	if msg.type != VEHICLE_TYPE_QUAD or msg.autopilot != AUTOPILOT_ARDUPILOT:
		continue

	# Get the current flight mode
	mode = mavutil.mode_string_v10(msg)

	# Get the vehicle armed status
	if vehicle.motors_armed():
		armed = "ARMED"
	else:
		armed = "DISARMED"

	# Get the battery voltage in Volts
	msg = vehicle.recv_match(type='SYS_STATUS', blocking=True)
	batt_volt_V = msg.voltage_battery * 0.001

	# Get the battery percentage
	msg = vehicle.recv_match(type='BATTERY_STATUS', blocking=True)
	batt_percentage = msg.battery_remaining

	print("%s; Mode: %s; Battery voltage: %.2f V; Battery percentage: %d" % (armed, mode, batt_volt_V, batt_percentage))

	# Write the vehicle status
	f = open("/var/www/html/uav-latest-status", "w")
	f.write(armed+"\nMode: "+mode+"\nBattery voltage: "+str(round(batt_volt_V*100)/100.0)+" V"+"\nBattery %: "+str(batt_percentage))
	f.close()

	if prev_armed != armed:
		f = open("/etc/default/xoss-armed-status", "w")
		f.write(armed+"\n")
		f.close()

	prev_armed=armed
	time.sleep(1)



