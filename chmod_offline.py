#!/usr/bin/env python3

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
	print("\t Too much arguments!")
	sys.exit(1)

# Get the target mode
target_mode = sys.argv[1].upper()

if num_args == 3:
	# Get the source mode
	source_mode = sys.argv[2].upper()

# MAVLINK variables initialization
HERELINK_SYS_ID = 42
AUTOPILOT_ARDUPILOT = 3
VEHICLE_TYPE_QUAD = 2

# Variables initialization
vehicle_detected_in_SOURCE_mode = False
max_establish_conn_count = 10
max_establish_conn_retry_attemts = 3
establish_conn_retry_attemts = 0
establish_conn_count = 0
vehicle_sys_id = 0
vehicle_component_id = 0
mode_id_obtained = 0

# Device initialization
device = '/dev/ttyTHS1' # Create a connection with the UART device
#device = 'udpin:localhost:5001' # Create a connection listening to a local UDP port (for debug only)

while True:
	try:
		if establish_conn_retry_attemts == 0:
			print("\t Establishing connection with the vehicle...")
			vehicle = mavutil.mavlink_connection(device,baud=115200)

		# Initialize the mode IDs
		while mode_id_obtained == 0:
			vehicle.wait_heartbeat()
			msg = vehicle.recv_match(type='HEARTBEAT',blocking=True)

			if msg.type == VEHICLE_TYPE_QUAD and msg.autopilot == AUTOPILOT_ARDUPILOT:
				mode = mavutil.mode_string_v10(msg)
				mode_id = vehicle.mode_mapping()[mode]

				if num_args == 2:
					# Set the source mode equal to the current mode
					source_mode = mode

				source_mode_id = vehicle.mode_mapping()[source_mode]
				target_mode_id = vehicle.mode_mapping()[target_mode]
				mode_id_obtained = 1
				establish_conn_count = 0
				break
			
			establish_conn_count = establish_conn_count + 1

			if establish_conn_count > 3 * max_establish_conn_count:
				print("\t Establishing connection failed (no valid HEARTBEET detected)! Mode change has been canceled!")
				sys.exit(1)
		
		# Initialize the vehicle system and component IDs and check if HERELINK is connected
		while establish_conn_count < max_establish_conn_count:
			vehicle = mavutil.mavlink_connection(device,baud=115200) # Has to be generated each iteration otherwise a system ID can be missed
			vehicle.wait_heartbeat() # This sets the system and component ID

			if vehicle.target_system == HERELINK_SYS_ID:
				print("\t HERELINK remote detected! Mode change has been canceled!")
				sys.exit(0)
			elif vehicle.target_system > 0 and vehicle.target_system < 256:
				vehicle_sys_id = vehicle.target_system
				vehicle_component_id = vehicle.target_component

			establish_conn_count = establish_conn_count + 1
		
		# Ensuring that the vehicle object can generate valid heartbeet messages
		establish_conn_count = 0
		while True:
			vehicle = mavutil.mavlink_connection(device,baud=115200)
			vehicle.wait_heartbeat()
			msg = vehicle.recv_match(type='HEARTBEAT',blocking=True)

			if msg.type == VEHICLE_TYPE_QUAD and msg.autopilot == AUTOPILOT_ARDUPILOT:
				break

			establish_conn_count = establish_conn_count + 1

			if establish_conn_count > 3 * max_establish_conn_count:
				print("\t Establishing connection failed! Mode change has been canceled!")
				sys.exit(1)

		if vehicle_sys_id > 0:
			print("\t Connection established (vehicle id: %u, component %u). Current vehicle mode is %s" % (vehicle_sys_id, vehicle_component_id, mode))
		else:
			establish_conn_retry_attemts = establish_conn_retry_attemts + 1

			if establish_conn_retry_attemts > max_establish_conn_retry_attemts:
				print("\t Establishing connection failed! Mode change has been canceled!")
				sys.exit(1)

			print("\t Establishing connection failed! Retrying...")
			establish_conn_count = 0
			continue

		if mode_id == source_mode_id or vehicle_detected_in_SOURCE_mode == True:
			
			vehicle_detected_in_SOURCE_mode = True
			
			while target_mode_id != mode_id:
				# Set the target mode
				print("\t Switching vehicle to mode %s. Sending mode change command..." % (target_mode))

				# DEPRECATED COMMAND: vehicle.mav.set_mode_send(vehicle_target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,new_mode_id)
				vehicle.mav.command_long_send(vehicle_sys_id, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, target_mode_id, 0, 0, 0, 0, 0)
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

			print("\t Vehicle mode successfully set to %s!" % (mode))
		else:
			print("\t Vehicle mode change canceled because vehicle was not in %s mode. Current vehicle mode is %s!" % (source_mode, mode))

		break
	except serial.serialutil.SerialException:
		print("\t Connection failed. Retrying...")
		time.sleep(2)
	except KeyError:
		print("\t Unknown mode %s specified. Mode change failed!" % (target_mode))
		sys.exit(1)
