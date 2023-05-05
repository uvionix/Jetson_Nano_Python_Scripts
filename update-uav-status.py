#!/usr/bin/env python3

import sys
import threading
import subprocess
import socket
import os
from time import sleep
from pymavlink import mavutil

SYSTEM_SERVICES_PATH = '/etc/systemd/system/'
CAMERA_SERVICE_FILE = 'camera-start@.service'
SHUTDOWN_SERVICE_FILE = 'system-shutdown.service'

api_commands = {"CMD_SET_FLIGHT_MODE"				: "setmode",
				"CMD_GET_FLIGHT_MODE"				: "getmode",
				"CMD_GET_SYSTEM_TIME"				: "gettime"}

api_responces = {"CMD_SUCCESSFULL"					: "ok",
		 		 "CMD_UNRECOGNIZED"					: "unrecognized command",
				 "CMD_FAILED"						: "error",
				 "CMD_UNRECOGNIZED_MODE"			: "unrecognized mode"}

log_messages = {"MSG_START_NEW_LOG"					: "============= INITIALIZING NEW LOG FILE =============",
				"MSG_INIT_ENV_VARS"					: "Initializing environment variables...",
				"MSG_HERELINK_ID"					: "Herelink sys ID set to",
				"MSG_AUTOPILOT_ID"					: "Autopilot Ardupilot ID set to",
				"MSG_VEHICLE_TYPE_ID"				: "Vehicle type quad ID set to",
				"MSG_RC_RECEIVER_CONNECTED_ID"		: "System status RC receiver connected ID set to",
				"MSG_RC_CHANNEL_MIN_PWM"			: "Minimum valid PWM for the RC channels set to",
				"MSG_SYSTEM_SHUTDOWN_RC_CH"			: "System shutdown RC channel set to",
				"MSG_CAMERA_REC_TRIG_RC_CH"			: "Camera record trigger RC channel set to",
				"MSG_CAM_CMDS_SOCKET_PORT"			: "Camera commands socket port set to",
				"MSG_COM_SERVER_SOCKET_PORT"		: "Communication server socket port set to",
				"MSG_COM_SERVER_SOCKET_RECVBUFF_SZ"	: "Communication server socket receive buffer size set to",
				"MSG_ERR_INIT_ENV_VARS"				: "Error initializing from the system environment variables",
				"MSG_INIT_CAM_SERVICE_SETUP_FILE"	: "Initializing camera service setup file...",
				"MSG_GSTREAMER_SETUP_FILE"			: "GStreamer setup file initialized to",
				"MSG_ERR_OPENING_FILE"				: "Error opening file",
				"MSG_INIT_CAM_CMDS"					: "Initializing camera commands...",
				"MSG_CAM_QUIT_SERVICE_HOTKEY"		: "Camera quit service hotkey set to",
				"MSG_CAM_START_SERVICE_HOTKEY"		: "Camera start service hotkey set to",
				"MSG_CAM_START_REC_HOTKEY"			: "Camera start recording hotkey set to",
				"MSG_CAM_STOP_REC_HOTKEY"			: "Camera stop recording hotkey set to",
				"MSG_CAM_REC_TRIG_DISABLED"			: "Camera recording trigger is disabled",
				"MSG_RC_CHANNELS_CONFIG_NOT_OK"		: "RC channels are not properly configured!",
				"MSG_NOT_ALL_PARAMS_INITIALIZED"	: "not all related parameters could be initialized!",
				"MSG_RC_SYSTEM_SHUTDOWN_ENABLED"	: "System shutdown via RC is enabled",
				"MSG_RC_SYSTEM_SHUTDOWN_DISABLED"	: "System shutdown via RC is disabled",
				"MSG_SETTING_NOT_FOUND"				: "cannot find setting as an environment variable!",
				"MSG_CONNECTING_TO_VEHICLE"			: "Connecting to vehicle...",
				"MSG_CONNECTION_ESTABLISHED"		: "Connection established",
				"MSG_STARTING_THREADS"				: "Starting threads...",
				"MSG_CAM_START_HOTKEY_NOT_INIT"		: "Camera service start hotkey not initialized! Camera service will not be started!",
				"MSG_ERR_CMD_SEND_FAILED"			: "Failed sending command",
				"MSG_STARTING_SHUTDOWN_SERVICE"		: "Starting system shutdown service...",
				"MSG_ERR_STARTING_SHUTDOWN_SERVICE"	: "Failed starting the system shutdown service!",
				"MSG_SHUTDOWN_SERVICE_STARTED"		: "System shutdown service started",
				"MSG_STOPPING_CAMERA"				: "Stopping camera service...",
				"MSG_START_RECORDING"				: "Start recording requested...",
				"MSG_STOP_RECORDING"				: "Stop recording requested...",
				"MSG_RC_CONTROLLER_CONNECTED"		: "Remote controller connected",
				"MSG_RC_CONTROLLER_DISCONNECTED"	: "Remote controller disconnected",
				"MSG_STARTING_CAMERA"				: "Connection with camera communication socket established. Starting camera...",
				"MSG_WAITING_FOR_CAM_SOCKET_CONN"	: "Waiting for camera communication socket connection...",
				"MSG_CAM_SOCKET_CONN_TERMINATED"	: "Connection with camera terminated!",
				"MSG_COM_SOCKET_ACCEPTED_CONN"		: "Communication server accepted connection:",
				"MSG_COM_SOCKET_CLOSED_CONN"		: "Communication server closed connection:",
				"MSG_ABORT"							: "Aborting..."}

class UavStatus:
	# Constructor
	def __init__(self, local_port, baudrate, status_file):
		self.__dt = 1.0
		self.__local_port = local_port
		self.__baudrate = baudrate
		self.__status_file = status_file
		self.__herelink_sys_id = 0
		self.__vehicle_sys_id = 0
		self.__vehicle_component_id = 0
		self.__autopilot_ardupilot = 0
		self.__vehicle_type_quad = 0
		self.__mav_sys_status_sensor_rc_receiver = 0
		self.__rc_channel_min_pwm_valid = 0
		self.__camera_socket_cmd_port = 0
		self.__com_server_socket_port = 0
		self.__com_server_socket_recv_buff_size = 0
		self.__log_file = '/var/log/uav-status'
		self.__armed = 'DISARMED'
		self.__mode = 'UNKNOWN'
		self.__target_mode = 'UNKNOWN'
		self.__timestamp = '[]'
		self.__batt_volt_V = 0.0
		self.__batt_percentage = 0
		self.__rec_channel_pwm = 0
		self.__shutdown_channel_pwm = 0
		self.__sys_shutdown_requested = False
		self.__start_recording = False
		self.__rc_receiver_connected = False
		self.__rc_system_shutdown_enabled = False
		self.__cam_com_socket_connecting = False
		self.__cam_com_socket_connected = False
		self.__gst_setup_file_initialized = False
		self.__camera_start_rec_hotkey_initialized = False
		self.__camera_stop_rec_hotkey_initialized = False
		self.__camera_start_hotkey_initialized = False
		self.__camera_quit_hotkey_initialized = False
		self.__cam_rec_trigger_rc_ch = -1
		self.__system_shutdown_rc_ch = -1
		self.__rec_enabled = False
		self.__prev_rec_channel_pwm = self.__rec_channel_pwm
		self.__prev_shutdown_channel_pwm = self.__shutdown_channel_pwm
		self.__prev_armed = self.__armed
		self.__prev_mode = self.__mode
		self.__prev_batt_volt_V = self.__batt_volt_V
		self.__prev_batt_percentage = self.__batt_percentage

		# Initialize the log file
		try:
			# Check if the log file can be initialized from an environment variable
			self.__log_file = os.environ['UAV_STATUS_LOG_FILE']
		except KeyError:
			pass
		
		# Clear the log file and start a new log
		with open(self.__log_file, 'w') as log:
			print(f"{log_messages['MSG_START_NEW_LOG']}", file=log)

		# Initialize from the system environment variables
		with open(self.__log_file, 'a') as log:
			print(f"{log_messages['MSG_INIT_ENV_VARS']}", file=log)
			try:
				self.__herelink_sys_id = int(os.environ['HERELINK_SYS_ID'])
				self.__autopilot_ardupilot = int(os.environ['AUTOPILOT_ARDUPILOT'])
				self.__vehicle_type_quad = int(os.environ['VEHICLE_TYPE_QUAD'])
				self.__mav_sys_status_sensor_rc_receiver = int(os.environ['MAV_SYS_STATUS_SENSOR_RC_RECEIVER'])
				self.__rc_channel_min_pwm_valid = int(os.environ['RC_CHANNEL_MIN_PWM_VALID'])
				self.__system_shutdown_rc_ch = int(os.environ['SYSTEM_SHUTDOWN_RC_CHANNEL'])
				self.__cam_rec_trigger_rc_ch = int(os.environ['CAMERA_REC_TRIGGER_RC_CHANNEL'])
				self.__camera_socket_cmd_port = int(os.environ['CAMERA_SOCKET_CMD_PORT'])
				self.__com_server_socket_port = int(os.environ['COM_SERVER_SOCKET_PORT'])
				self.__com_server_socket_recv_buff_size = int(os.environ['COM_SERVER_SOCKET_RECV_BUFF_SIZE'])

				print(f"\t {log_messages['MSG_HERELINK_ID']} {self.__herelink_sys_id}", file=log)
				print(f"\t {log_messages['MSG_AUTOPILOT_ID']} {self.__autopilot_ardupilot}", file=log)
				print(f"\t {log_messages['MSG_VEHICLE_TYPE_ID']} {self.__vehicle_type_quad}", file=log)
				print(f"\t {log_messages['MSG_RC_RECEIVER_CONNECTED_ID']} {self.__mav_sys_status_sensor_rc_receiver}", file=log)
				print(f"\t {log_messages['MSG_RC_CHANNEL_MIN_PWM']} {self.__rc_channel_min_pwm_valid}", file=log)
				print(f"\t {log_messages['MSG_SYSTEM_SHUTDOWN_RC_CH']} {self.__system_shutdown_rc_ch}", file=log)
				print(f"\t {log_messages['MSG_CAMERA_REC_TRIG_RC_CH']} {self.__cam_rec_trigger_rc_ch}", file=log)
				print(f"\t {log_messages['MSG_CAM_CMDS_SOCKET_PORT']} {self.__camera_socket_cmd_port}", file=log)
				print(f"\t {log_messages['MSG_COM_SERVER_SOCKET_PORT']} {self.__com_server_socket_port}", file=log)
				print(f"\t {log_messages['MSG_COM_SERVER_SOCKET_RECVBUFF_SZ']} {self.__com_server_socket_recv_buff_size}", file=log)
			except KeyError:
				print(f"{log_messages['MSG_ERR_INIT_ENV_VARS']}", file=log)
				print(f"{log_messages['MSG_ABORT']}", file=log)
				sys.exit(1)

		# Get the setup file for the camera service
		with open(self.__log_file, 'a') as log:
			print(f"{log_messages['MSG_INIT_CAM_SERVICE_SETUP_FILE']}", file=log)
			try:
				with open(SYSTEM_SERVICES_PATH+CAMERA_SERVICE_FILE, 'r') as f:
					for line in f.readlines():
						try:
							if line.split(sep='=')[0].lower() == 'environmentfile':
								self.__gst_setup_file = line.split(sep='=')[1].strip(' \n')
								print(f"\t {log_messages['MSG_GSTREAMER_SETUP_FILE']} {self.__gst_setup_file}", file=log)
								self.__gst_setup_file_initialized = True
								break
						except IndexError:
							continue
			except IOError:
				print(f"{log_messages['MSG_ERR_OPENING_FILE']} {SYSTEM_SERVICES_PATH+CAMERA_SERVICE_FILE}", file=log)

		# Get the camera commands and initialize the related parameters
		with open(self.__log_file, 'a') as log:
			if self.__gst_setup_file_initialized:
				print(f"{log_messages['MSG_INIT_CAM_CMDS']}", file=log)
				try:
					with open(self.__gst_setup_file, 'r') as f:
						for line in f.readlines():
							try:
								param = line.split(sep='"')[0]
								if param.upper() == 'CAMERA_QUIT_SERVICE_HOTKEY=':
									self.__camera_quit_hotkey = line.split(sep='"')[1]
									self.__camera_quit_hotkey_initialized = True
									print(f"\t {log_messages['MSG_CAM_QUIT_SERVICE_HOTKEY']} {self.__camera_quit_hotkey}", file=log)
								elif param.upper() == 'CAMERA_START_SERVICE_HOTKEY=':
									self.__camera_start_hotkey = line.split(sep='"')[1]
									self.__camera_start_hotkey_initialized = True
									print(f"\t {log_messages['MSG_CAM_START_SERVICE_HOTKEY']} {self.__camera_start_hotkey}", file=log)
								elif param.upper() == 'CAMERA_START_REC_HOTKEY=':
									self.__camera_start_rec_hotkey = line.split(sep='"')[1]
									self.__camera_start_rec_hotkey_initialized = True
									print(f"\t {log_messages['MSG_CAM_START_REC_HOTKEY']} {self.__camera_start_rec_hotkey}", file=log)
								elif param.upper() == 'CAMERA_STOP_REC_HOTKEY=':
									self.__camera_stop_rec_hotkey = line.split(sep='"')[1]
									self.__camera_stop_rec_hotkey_initialized = True
									print(f"\t {log_messages['MSG_CAM_STOP_REC_HOTKEY']} {self.__camera_stop_rec_hotkey}", file=log)
							except IndexError:
								continue
				except IOError:
					print(f"{log_messages['MSG_ERR_OPENING_FILE']} {self.__gst_setup_file}", file=log)

			if self.__camera_quit_hotkey_initialized and self.__cam_rec_trigger_rc_ch >= 0 and \
				self.__camera_start_rec_hotkey_initialized and self.__camera_stop_rec_hotkey_initialized:
				if self.__system_shutdown_rc_ch != self.__cam_rec_trigger_rc_ch:
					self.__rec_enabled = True
				else:
					print(f"{log_messages['MSG_CAM_REC_TRIG_DISABLED']} - {log_messages['MSG_RC_CHANNELS_CONFIG_NOT_OK']}", file=log)
			else:
				print(f"{log_messages['MSG_CAM_REC_TRIG_DISABLED']} - {log_messages['MSG_NOT_ALL_PARAMS_INITIALIZED']}", file=log)

			try:
				if int(os.environ['SYSTEM_SHUTDOWN_VIA_RC_ENABLED']):
					if self.__system_shutdown_rc_ch >=0 and self.__system_shutdown_rc_ch != self.__cam_rec_trigger_rc_ch:
						print(f"{log_messages['MSG_RC_SYSTEM_SHUTDOWN_ENABLED']}", file=log)
						self.__rc_system_shutdown_enabled = True
					else:
						print(f"{log_messages['MSG_RC_SYSTEM_SHUTDOWN_DISABLED']} - {log_messages['MSG_RC_CHANNELS_CONFIG_NOT_OK']}", file=log)
				else:
					print(f"{log_messages['MSG_RC_SYSTEM_SHUTDOWN_DISABLED']}", file=log)
			except KeyError:
				print(f"{log_messages['MSG_RC_SYSTEM_SHUTDOWN_DISABLED']} - {log_messages['MSG_SETTING_NOT_FOUND']}", file=log)

		# Create the camera communication socket
		self.__cam_com_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		# Create a connection listening to a local UDP port (MAVProxy service has to be running)
		with open(self.__log_file, 'a') as log:
			print(f"{log_messages['MSG_CONNECTING_TO_VEHICLE']}", file=log)
			while self.__vehicle_sys_id == self.__herelink_sys_id or self.__vehicle_sys_id == 0:
				self.__vehicle = mavutil.mavlink_connection(device='udpin:localhost:'+str(self.__local_port), baud=self.__baudrate)
				self.__vehicle.wait_heartbeat()
				self.__vehicle_sys_id = self.__vehicle.target_system
				self.__vehicle_component_id = self.__vehicle.target_component

			print(f"\t {log_messages['MSG_CONNECTION_ESTABLISHED']} - vehicle system ID: {self.__vehicle_sys_id}, vehicle component ID: {self.__vehicle_component_id}", file=log)
		
		# Initialize the mode mapping dictionary
		self.__vehicle_mode_map = self.__vehicle.mode_mapping().copy()

		# Create the threads
		self.__threadLock = threading.Lock()
		self.__t1 = threading.Thread(target=self.__get_uav_status__, args=())
		self.__t2 = threading.Thread(target=self.__update_uav_status__, args=())
		self.__t3 = threading.Thread(target=self.__cam_com_socket_connect__, args=())
		self.__t4 = threading.Thread(target=self.__com_socket_server__, args=())
	
	# Start the threads
	def start_threads(self):
		with open(self.__log_file, 'a') as log:
			print(f"{log_messages['MSG_STARTING_THREADS']}", file=log)
			if self.__camera_start_hotkey_initialized:
				self.__t3.start()
			else:
				print(f"\t {log_messages['MSG_CAM_START_HOTKEY_NOT_INIT']}", file=log)

		self.__t1.start()
		self.__t2.start()
		self.__t4.start()

	# Get the UAV armed/disarmed status
	def get_armed_status(self):
		return self.__armed

	# Get the UAV mode
	def get_mode(self):
		return self.__mode

	# Get the battery percentage
	def get_batt_percentage(self):
		return self.__batt_percentage
	
	# Get the battery voltage
	def get_batt_voltage(self):
		return self.__batt_volt_V
	
	# Send command to the camera
	def cam_send_command(self, cmd):
		try:
			self.__cam_com_socket.sendall(bytes(cmd+"\n","utf-8"))
		except socket.error:
			with open(self.__log_file, 'a') as log:
				print(f"{self.__timestamp} {log_messages['MSG_ERR_CMD_SEND_FAILED']} {cmd}!", file=log)

	# Reset the RC channels history
	def reset_rc_channels_history(self):
		self.__prev_shutdown_channel_pwm = 0
		self.__prev_rec_channel_pwm = 0

	# Get the UAV status by listening for specific MAVLINK messages
	def __get_uav_status__(self):
		while True:
			# Get a MAVLINK message
			msg = self.__vehicle.recv_match(type=['HEARTBEAT','RC_CHANNELS','SYS_STATUS','BATTERY_STATUS'], blocking=True)
			msg_type = msg.get_type()

			if msg_type == 'HEARTBEAT':
				if msg.type == self.__vehicle_type_quad and msg.autopilot == self.__autopilot_ardupilot:
					# Get the vehicle mode from the HEARTBEAT message
					if self.__threadLock.acquire():
						new_mode = mavutil.mode_string_v10(msg)
						if new_mode != self.__mode:
							# Reset the target mode if the vehicle mode has changed
							self.__target_mode = new_mode

						self.__mode = new_mode

						if self.__target_mode in self.__vehicle.mode_mapping() and self.__target_mode != self.__mode:
							target_mode_id = self.__vehicle.mode_mapping()[self.__target_mode]
							self.__vehicle.mav.command_long_send(self.__vehicle_sys_id, self.__vehicle_component_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, target_mode_id, 0, 0, 0, 0, 0)
							with open(self.__log_file, 'a') as log:
								print(f"{self.__timestamp} Switching vehicle to mode {self.__target_mode}... (current mode is {self.__mode})", file=log)

						self.__threadLock.release()
			elif self.__rc_receiver_connected and msg_type == 'RC_CHANNELS':
				if self.__rc_system_shutdown_enabled:
					if self.__threadLock.acquire():
						# Initiate system shutdown upon toggle of the respective signal
						self.__shutdown_channel_pwm = eval("msg.chan"+str(self.__system_shutdown_rc_ch)+"_raw")

						if self.__shutdown_channel_pwm > self.__rc_channel_min_pwm_valid:
							if self.__prev_shutdown_channel_pwm != 0 and \
								self.__prev_shutdown_channel_pwm != self.__shutdown_channel_pwm and \
									self.__armed == "DISARMED" and self.__sys_shutdown_requested == False:
								with open(self.__log_file, 'a') as log:
									print(f"{self.__timestamp} {log_messages['MSG_STARTING_SHUTDOWN_SERVICE']}", file=log)
									if subprocess.call(['systemctl','start',SHUTDOWN_SERVICE_FILE]):
										print(f"\t {log_messages['MSG_ERR_STARTING_SHUTDOWN_SERVICE']}", file=log)
									else:
										self.__sys_shutdown_requested = True
										print(f"\t {log_messages['MSG_SHUTDOWN_SERVICE_STARTED']}", file=log)

							self.__prev_shutdown_channel_pwm = self.__shutdown_channel_pwm

						self.__threadLock.release()

				if self.__rec_enabled and self.__cam_com_socket_connected:
					# Stop the recording if system shutdown is requested
					if self.__sys_shutdown_requested:
						if self.__threadLock.acquire():
							with open(self.__log_file, 'a') as log:
								print(f"{self.__timestamp} {log_messages['MSG_STOPPING_CAMERA']}", file=log)
							
							self.cam_send_command(cmd=self.__camera_stop_rec_hotkey)
							sleep(3)
							self.cam_send_command(cmd=self.__camera_quit_hotkey)
							self.__rec_enabled = False
							self.__threadLock.release()
							continue

					# Update the recording state from the RC_CHANNELS message
					self.__rec_channel_pwm = eval("msg.chan"+str(self.__cam_rec_trigger_rc_ch)+"_raw")

					if self.__rec_channel_pwm < self.__rc_channel_min_pwm_valid:
						continue

					if self.__prev_rec_channel_pwm == 0:
						self.__prev_rec_channel_pwm = self.__rec_channel_pwm
						continue

					if self.__threadLock.acquire():
						if self.__prev_rec_channel_pwm != self.__rec_channel_pwm:
							# Toggle the recording state
							self.__start_recording = not(self.__start_recording)

							with open(self.__log_file, 'a') as log:
								if self.__start_recording:
									print(f"{self.__timestamp} {log_messages['MSG_START_RECORDING']}", file=log)
									rec_cmd = self.__camera_start_rec_hotkey
								else:
									print(f"{self.__timestamp} {log_messages['MSG_STOP_RECORDING']}", file=log)
									rec_cmd = self.__camera_stop_rec_hotkey

							# Send the recording command
							self.cam_send_command(cmd=rec_cmd)

						self.__prev_rec_channel_pwm = self.__rec_channel_pwm
						self.__threadLock.release()
			elif msg_type == 'SYS_STATUS':
				if self.__threadLock.acquire():
					# Check if the remote controller is connected
					if msg.onboard_control_sensors_health & self.__mav_sys_status_sensor_rc_receiver:
						if self.__rc_receiver_connected == False:
							self.__rc_receiver_connected = True
							with open(self.__log_file, 'a') as log:
								print(f"{self.__timestamp} {log_messages['MSG_RC_CONTROLLER_CONNECTED']}", file=log)
					elif self.__rc_receiver_connected == True:
						self.__rc_receiver_connected = False
						self.reset_rc_channels_history()
						with open(self.__log_file, 'a') as log:
							print(f"{self.__timestamp} {log_messages['MSG_RC_CONTROLLER_DISCONNECTED']}", file=log)

					# Get the battery voltage
					self.__batt_volt_V = round((msg.voltage_battery * 0.001)*10)/10
					self.__threadLock.release()
			elif msg_type == 'BATTERY_STATUS':
				# Get the battery percentage from the BATTERY_STATUS message
				if self.__threadLock.acquire():
					self.__batt_percentage = msg.battery_remaining
					self.__threadLock.release()
	
	# Update the UAV status by writing the status variables to file
	def __update_uav_status__(self):
		while True:
			if self.__threadLock.acquire():
				# Update the armed/disarmed status
				if self.__vehicle.motors_armed():
					self.__armed = "ARMED"
				else:
					self.__armed = "DISARMED"

				# Write the status to a file if there is a change
				if (self.__prev_mode != self.__mode) or \
					(self.__prev_batt_volt_V > self.__batt_volt_V) or \
						(self.__prev_batt_percentage != self.__batt_percentage) or \
							(self.__prev_armed != self.__armed):
					self.__prev_mode = self.__mode
					self.__prev_batt_volt_V = self.__batt_volt_V
					self.__prev_batt_percentage = self.__batt_percentage
					self.__prev_armed = self.__armed

					with open(self.__log_file, 'a') as log:
						print(f"{self.__timestamp} {self.__armed}; Mode: {self.__mode}; Battery voltage: {self.__batt_volt_V} V; Battery percentage: {self.__batt_percentage}", file=log)

						try:
							f = open(self.__status_file, "w")
							f.write(self.__armed+"\nMode: "+self.__mode+"\nBattery voltage: "+str(self.__batt_volt_V)+" V"+"\nBattery %: "+str(self.__batt_percentage)+"\n")
							f.close()
						except IOError:
							print(f"{self.__timestamp} {log_messages['MSG_ERR_OPENING_FILE']} {self.__status_file}", file=log)

				self.__threadLock.release()

			sleep(self.__dt)
	
	# Connect to the camera communication socket
	def __cam_com_socket_connect__(self):
		while True:
			if self.__cam_com_socket_connected == False and self.__mode != 'UNKNOWN' and self.__sys_shutdown_requested == False:
				if self.__threadLock.acquire():
					try:
						self.__cam_com_socket.connect(('localhost', self.__camera_socket_cmd_port))
						self.__cam_com_socket_connected = True
						self.__cam_com_socket_connecting = False
						with open(self.__log_file, 'a') as log:
							print(f"{self.__timestamp} {log_messages['MSG_STARTING_CAMERA']}", file=log)
					except socket.error:
						if self.__cam_com_socket_connecting == False:
							self.__cam_com_socket_connecting = True
							with open(self.__log_file, 'a') as log:
								print(f"{self.__timestamp} {log_messages['MSG_WAITING_FOR_CAM_SOCKET_CONN']}", file=log)

					self.__threadLock.release()
			elif self.__cam_com_socket_connected == True:
				if self.__threadLock.acquire():
					try:
						self.__cam_com_socket.sendall(bytes(self.__camera_start_hotkey+"\n","utf-8"))
					except socket.error:
						self.__cam_com_socket_connected = False
						self.__cam_com_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
						with open(self.__log_file, 'a') as log:
							print(f"{self.__timestamp} {log_messages['MSG_CAM_SOCKET_CONN_TERMINATED']}", file=log)
					
					self.__threadLock.release()

			sleep(self.__dt)

	# Communication socket server
	def __com_socket_server__(self):
		while True:
			with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
				srv.bind(('localhost', self.__com_server_socket_port))
				srv.listen()
				conn, addr = srv.accept()
				with conn:
					with open(self.__log_file, 'a') as log:
						print(f"{self.__timestamp} {log_messages['MSG_COM_SOCKET_ACCEPTED_CONN']} {addr}", file=log)
					while True:
						data = conn.recv(self.__com_server_socket_recv_buff_size)
						if not data:
							with open(self.__log_file, 'a') as log:
								print(f"{self.__timestamp} {log_messages['MSG_COM_SOCKET_CLOSED_CONN']} {addr}", file=log)
							break
						
						# Parse the input data
						data_str = str(data, "utf-8").strip()

						# Get the command
						cmd = data_str.split(sep=' ')[0].lower()

						# Get the argument
						try:
							arg = data_str.split(sep=' ')[1]
						except IndexError:
							arg=''
						
						reply = api_responces["CMD_FAILED"]
						if cmd == api_commands["CMD_SET_FLIGHT_MODE"]:
							new_mode = arg.upper()
							if new_mode in self.__vehicle_mode_map:
								if self.__threadLock.acquire():
									self.__target_mode = new_mode
									self.__threadLock.release()
								
									while new_mode != self.__mode:
										pass
									reply = api_responces["CMD_SUCCESSFULL"]
							else:
								reply = api_responces["CMD_UNRECOGNIZED_MODE"]
						elif cmd == api_commands["CMD_GET_FLIGHT_MODE"]:
							reply = self.__mode
						elif cmd == api_commands["CMD_GET_SYSTEM_TIME"]:
							reply = '0'
						else:
							reply = api_responces["CMD_UNRECOGNIZED"]

						conn.sendall(bytes(reply,"utf-8"))

# Create the vehicle status object and start execution
status = UavStatus(local_port=int(sys.argv[1]), baudrate=int(sys.argv[2]), status_file=sys.argv[3])
status.start_threads()

# Start main thread
while True:
	pass
