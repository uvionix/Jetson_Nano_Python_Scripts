#!/usr/bin/env python3

import sys
import threading
from time import sleep
from pymavlink import mavutil

AUTOPILOT_ARDUPILOT = 3
VEHICLE_TYPE_QUAD = 2
GST_SERVICE_FILE = "/etc/systemd/system/gstreamer-autostart.service"

class UavStatus:
	# Constructor
	def __init__(self, local_port, baudrate, status_file):
		self.__dt = 1.0
		self.__local_port = local_port
		self.__baudrate = baudrate
		self.__status_file = status_file
		self.__armed = 'DISARMED'
		self.__mode = 'UNKNOWN'
		self.__batt_volt_V = 0.0
		self.__batt_percentage = 0
		self.__rec_channel_pwm = 0
		self.__start_recording = False
		self.__camera_cmds_file_initialized = False
		self.__gst_setup_file_initialized = False
		self.__camera_start_rec_hotkey_initialized = False
		self.__camera_stop_rec_hotkey_initialized = False
		self.__cam_rec_trigger_ch = -1
		self.__rec_enabled = False
		self.__prev_rec_channel_pwm = self.__rec_channel_pwm
		self.__prev_armed = self.__armed
		self.__prev_mode = self.__mode
		self.__prev_batt_volt_V = self.__batt_volt_V
		self.__prev_batt_percentage = self.__batt_percentage

		# Get the setup file for the gstreamer service
		try:
			with open(GST_SERVICE_FILE, 'r') as f:
				for line in f.readlines():
					try:
						if line.split(sep='=')[0].lower() == 'environmentfile':
							self.__gst_setup_file = line.split(sep='=')[1].strip(' \n')
							print("Gstreamer setup file initialized to %s" % self.__gst_setup_file)
							self.__gst_setup_file_initialized = True
							break
					except IndexError:
						continue
		except IOError:
			print("Error opening file %s" % GST_SERVICE_FILE)

		# Get the camera commands file and initialize the related paramters
		if self.__gst_setup_file_initialized:
			try:
				with open(self.__gst_setup_file, 'r') as f:
					for line in f.readlines():
						try:
							param = line.split(sep='"')[0]
							if param.upper() == 'CMD_FILE=':
								self.__camera_cmds_file = line.split(sep='"')[1]
								self.__camera_cmds_file_initialized = True
								print("Camera commands file initialized to %s" % self.__camera_cmds_file)
							elif param.upper() == 'CAMERA_REC_TRIGGER_CHANNEL=':
								self.__cam_rec_trigger_ch = int(line.split(sep='"')[1])
								print("Camera record trigger channel initialized to %d" % self.__cam_rec_trigger_ch)
							elif param.upper() == 'CAMERA_START_REC_HOTKEY=':
								self.__camera_start_rec_hotkey = line.split(sep='"')[1]
								self.__camera_start_rec_hotkey_initialized = True
								print("Camera start recording hotkey set to %s" % self.__camera_start_rec_hotkey)
							elif param.upper() == 'CAMERA_STOP_REC_HOTKEY=':
								self.__camera_stop_rec_hotkey = line.split(sep='"')[1]
								self.__camera_stop_rec_hotkey_initialized = True
								print("Camera stop recording hotkey set to %s" % self.__camera_stop_rec_hotkey)
						except IndexError:
							continue
			except IOError:
				print("Error opening file %s" % self.__gst_setup_file)

		if self.__camera_cmds_file_initialized and self.__cam_rec_trigger_ch >= 0 and \
			self.__camera_start_rec_hotkey_initialized and self.__camera_stop_rec_hotkey_initialized:
			self.__rec_enabled = True
		else:
			print("Camera recording trigger is disabled! Not all related parameters could be initialized!")

		# Create a connection listening to a local UDP port (MAVProxy service has to be running)
		self.__vehicle = mavutil.mavlink_connection(device='udpin:localhost:'+str(self.__local_port), baud=self.__baudrate)
		self.__vehicle.wait_heartbeat()

		# Create the threads
		self.__threadLock = threading.Lock()
		self.__t1 = threading.Thread(target=self.__get_uav_status__, args=())
		self.__t2 = threading.Thread(target=self.__update_uav_status__, args=())
	
	# Start the threads
	def start_threads(self):
		self.__t1.start()
		self.__t2.start()

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

	# Get the UAV status by listening for specific MAVLINK messages
	def __get_uav_status__(self):
		while True:
			# Get a MAVLINK message
			msg = self.__vehicle.recv_match(type=['HEARTBEAT','RC_CHANNELS','SYS_STATUS','BATTERY_STATUS'], blocking=True)
			msg_type = msg.get_type()

			if msg_type == 'HEARTBEAT':
				if msg.type == VEHICLE_TYPE_QUAD and msg.autopilot == AUTOPILOT_ARDUPILOT:
					# Get the vehicle mode from the HEARTBEAT message
					if self.__threadLock.acquire():
						self.__mode = mavutil.mode_string_v10(msg)
						self.__threadLock.release()
			elif msg_type == 'RC_CHANNELS' and self.__rec_enabled:
				# Update the recording state from the RC_CHANNELS message
				self.__rec_channel_pwm = eval("msg.chan"+str(self.__cam_rec_trigger_ch)+"_raw")

				if self.__rec_channel_pwm < 980:
					continue

				if self.__prev_rec_channel_pwm == 0:
					self.__prev_rec_channel_pwm = self.__rec_channel_pwm
					continue

				if self.__prev_rec_channel_pwm != self.__rec_channel_pwm:
					# Toggle the recording state
					self.__start_recording = not(self.__start_recording)

					if self.__start_recording:
						print("Start recording")
						rec_cmd = self.__camera_start_rec_hotkey
					else:
						print("Stop recording")
						rec_cmd = self.__camera_stop_rec_hotkey

					# Send the recording command
					try:
						f = open(self.__camera_cmds_file, "a")
						f.write(rec_cmd+"\n")
						f.close()
					except IOError:
						print("Error opening file %s" % self.__camera_cmds_file)

				self.__prev_rec_channel_pwm = self.__rec_channel_pwm
				continue
			elif msg_type == 'SYS_STATUS':
				# Get the battery voltage from the SYS_STATUS message
				if self.__threadLock.acquire():
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
						(self.__prev_batt_percentage > self.__batt_percentage) or \
							(self.__prev_armed != self.__armed):
					self.__prev_mode = self.__mode
					self.__prev_batt_volt_V = self.__batt_volt_V
					self.__prev_batt_percentage = self.__batt_percentage
					self.__prev_armed = self.__armed

					print("%s; Mode: %s; Battery voltage: %.1f V; Battery percentage: %d" % (self.__armed, self.__mode, self.__batt_volt_V, self.__batt_percentage))

					try:
						f = open(self.__status_file, "w")
						f.write(self.__armed+"\nMode: "+self.__mode+"\nBattery voltage: "+str(self.__batt_volt_V)+" V"+"\nBattery %: "+str(self.__batt_percentage))
						f.close()
					except IOError:
						print("Error opening file %s" % self.__status_file)

				self.__threadLock.release()

			sleep(self.__dt)

# Create the vehicle status object and start execution
status = UavStatus(local_port=int(sys.argv[1]), baudrate=int(sys.argv[2]), status_file=sys.argv[3])
status.start_threads()

# Start main thread
while True:
	pass
