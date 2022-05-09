#!/usr/bin/env python

# The script is used to change the mode of the vehicle by establishing a connection with the UART device. This connection is possible
# when the UART device is not being accessed by another process i.e. MAVProxy. The MAVProxy process is disabled when the vehicle looses
# network connectivity. In that case this script can be triggered to switch the vehicle in a known mode (for example LOITER). 
# To run the script use one of the following commands
#	
#	python <script_name> <target_mode>
#	python <script_name> <target_mode> <source_mode>
#	python3 <script_name> <target_mode>
#	python3 <script_name> <target_mode> <source_mode>
# 
# where 
#
#	<script_name> is the name of this script (exaple: <script_name> = chmod_offline.py)
#	<target_mode> is the mode to which the vehicle should switch to (example: <target_mode> = loiter)
#	<source_mode> when provided is the mode in which the vehicle should be in order to switch to the provided target mode (example: <source_mode> = auto)
#	NOTE: If the vehicle is not in the source mode the mode change will be cancelled
#
# NOTE: The mode names have to be according to the ArduPilot names convention.

import sys
import time
import serial
from pymavlink import mavutil

# Check the number of command line arguments
num_args = len(sys.argv)

if num_args < 2:
	print("\t Not enough arguments!")
	sys.exit(1)
elif num_args > 3:
	print("\t Too much arguments")
	sys.exit(1)

# Get the target mode
target_mode = sys.argv[1].upper()

if num_args == 3:
	# Get the source mode
	source_mode = sys.argv[2].upper()

HERELINK_SYS_ID = 42
AUTOPILOT_ARDUPILOT = 3
VEHICLE_TYPE_QUAD = 2
vehicle_detected_in_SOURCE_mode = False

while True:
	try:
		while True:
			print("\t Establishing connection with the vehicle...")

			# Create a connection with the UART device
			vehicle = mavutil.mavlink_connection('/dev/ttyTHS1',baud=115200)

			# Create a connection listening to a local UDP port (for debug only)
			#vehicle = mavutil.mavlink_connection('udpin:localhost:5001',baud=115200)

			# Wait for the first heartbeat - this sets the system and component ID of remote system for the link
			vehicle.wait_heartbeat()
			msg = vehicle.recv_match(type='HEARTBEAT',blocking=True)
			
			if vehicle.target_system > 0 and vehicle.target_system != HERELINK_SYS_ID and msg.type == VEHICLE_TYPE_QUAD and msg.autopilot == AUTOPILOT_ARDUPILOT:
				break
			else:
				continue

		mode = mavutil.mode_string_v10(msg)
		mode_id = vehicle.mode_mapping()[mode]
		print("\t Connection established (vehicle id: %u, component %u). Current vehicle mode is %s" % (vehicle.target_system, vehicle.target_component, mode))

		if num_args == 2:
			# Set the source mode equal to the current mode
			source_mode = mode

		if mode_id == vehicle.mode_mapping()[source_mode] or vehicle_detected_in_SOURCE_mode == True:
			# Set the target mode and get a mode ID
			vehicle_detected_in_SOURCE_mode = True
			new_mode_id = vehicle.mode_mapping()[target_mode]
			
			while new_mode_id != mode_id:
				# Set the target mode
				print("\t Switching vehicle to mode %s. Sending mode change command..." % (target_mode))
				# DEPRECATED COMMAND: vehicle.mav.set_mode_send(vehicle_target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,new_mode_id)
				vehicle.mav.command_long_send(vehicle.target_system, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, new_mode_id, 0, 0, 0, 0, 0)
				print("\t Mode change command sent! Waiting for heartbeet...")
				time.sleep(2)
				
				# Update the vehicle mode
				while True:
					msg = vehicle.recv_match(type='HEARTBEAT',blocking=True)

					if msg.type != VEHICLE_TYPE_QUAD or msg.autopilot != AUTOPILOT_ARDUPILOT:
						continue

					break

				mode = mavutil.mode_string_v10(msg)
				mode_id = vehicle.mode_mapping()[mode]
				print("\t Heartbeat message received. Current vehicle mode is %s" % (mode))

			print("\t Vehicle mode successfully set to %s" % (mode))
		else:
			print("\t Vehicle mode change canceled because vehicle was not in %s mode. Current vehicle mode is %s" % (source_mode, mode))

		break
	except serial.serialutil.SerialException:
		print("\t Connection failed. Retrying...")
		time.sleep(2)
	except KeyError:
		print("\t Unknown mode %s specified. Mode change failed!" % (target_mode))
		sys.exit(1)
