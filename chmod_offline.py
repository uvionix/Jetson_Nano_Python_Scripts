#!/usr/bin/env python3

# The script is used to change the mode of the vehicle by establishing a connection with a communication socket,
# opened by a dedicated service which monitors the vehicle status. To run the script use one of the following commands:
#
#	python3 <script_name> <port> <target_mode>
#	python3 <script_name> <port> <target_mode> <source_mode>
#
# where 
#
#   <port> is the port on which the connection has to be established (example: <port> = 5001)
#	<script_name> is the name of this script (example: <script_name> = chmod_offline.py)
#	<target_mode> is the mode to which the vehicle should switch to (example: <target_mode> = loiter)
#	<source_mode> when provided is the mode in which the vehicle should be in, in order to switch to the provided target mode (example: <source_mode> = auto)
#	NOTE: If the vehicle is not in the source mode the mode change will be cancelled
#
# NOTE: The mode names have to be according to the ArduPilot names convention.

import sys
import socket
from time import sleep
from uvx_ipc_api import api_commands
from uvx_ipc_api import api_responses

SYSTEM_SERVICES_PATH = '/etc/systemd/system/'
MAVPROXY_SERVICE_FILE = 'mavproxy-autostart.service'

# Send a command and wait for a response. The response is returned
def send_command(com_socket, cmd, buff_size):
	while True:
		try:
			com_socket.sendall(bytes(cmd,"utf-8"))
		except socket.error:
			print(f"\t Error sending command {cmd}. Retrying...")
			sleep(2.0)
			continue
		
		# Get the command response
		cmd_res = str(com_socket.recv(buff_size),"utf-8")
		if cmd_res == api_responses["CMD_FAILED"]:
			print("\t Command failed. Retrying...")
			sleep(2.0)
		else:
			return cmd_res

# Get vehicle mode
def get_mode(com_socket):
	mode = send_command(com_socket=com_socket, cmd=api_commands["CMD_GET_FLIGHT_MODE"], buff_size=recv_buff_size)
	return mode.upper()

print("\t Starting mode change script...")

# Get the mavproxy setup file and initialize the receive buffer size, which specifies the maximum number of bytes to receive
recv_buff_size = 512
try:
	with open(SYSTEM_SERVICES_PATH+MAVPROXY_SERVICE_FILE, 'r') as f:
		for line in f.readlines():
			try:
				if line.split(sep='=')[0].lower() == 'environmentfile':
					mavproxy_setup_file = line.split(sep='=')[1].strip(' \n')
					try:
						with open(mavproxy_setup_file, 'r') as stp:
							for line in stp.readlines():
								try:
									param = line.split(sep='"')[0]
									if param.upper() == 'COM_SERVER_SOCKET_RECV_BUFF_SIZE=':
										try:
											recv_buff_size = int(line.split(sep='"')[1])
										except ValueError:
											print(f"\t Error parsing receive buffer size parameter from file {mavproxy_setup_file}")
											break
										break
								except IndexError:
									continue
					except IOError:
						print(f"\t Error opening file {mavproxy_setup_file}")
					
					break
			except IndexError:
				continue
except IOError:
	print(f"\t Error opening file {SYSTEM_SERVICES_PATH+MAVPROXY_SERVICE_FILE}")

# Check the number of command line arguments
num_args = len(sys.argv)

if num_args < 3:
	print("\t Not enough input arguments!")
	sys.exit(1)
elif num_args > 4:
	print("\t Too many input arguments!")
	sys.exit(1)

# Get the parameters
try:
    local_port = int(sys.argv[1])
except ValueError:
	print(f"\t Invalid port {sys.argv[1]} specified!")
	sys.exit(1)

target_mode = sys.argv[2].upper()

# Connect to the communication socket
cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
	print(f"\t Establishing socket connection on port {local_port} ...", end=' ')
	try:
		cli.connect(('localhost', local_port))
		print("Connection established!")
		break
	except ConnectionRefusedError:
		print("Connection refused. Retrying...")
		sleep(2.0)

# Get the current flight mode
mode = get_mode(com_socket=cli)
print(f"\t Current vehicle mode is {mode}")

# Get the source mode
if num_args == 4:
	source_mode = sys.argv[3].upper()
else:
	source_mode = mode

if mode == source_mode:
	while mode != target_mode:
		print(f"\t Switching vehicle to mode {target_mode}. Sending mode change command...")
		cmd_res = send_command(com_socket=cli, cmd=(api_commands["CMD_SET_FLIGHT_MODE"] + " " + target_mode), buff_size=recv_buff_size)
		
		if cmd_res == api_responses["CMD_SUCCESSFULL"]:
			sleep(3.0)
			mode = get_mode(com_socket=cli)
		elif cmd_res == api_responses["CMD_UNRECOGNIZED_MODE"]:
			print(f"\t Unknown mode {target_mode} specified. Mode change failed!")
			break
		elif cmd_res == api_responses["CMD_UNRECOGNIZED"]:
			print("\t Unknown mode change command specified. Mode change failed!")
			break
		else:
			print("\t Unexpected error occurred. Mode change failed!")
			break
	else:
		print(f"\t Vehicle mode successfully set to {mode}!")
else:
	print(f"\t Vehicle mode change canceled - vehicle was not in {source_mode} mode. Current vehicle mode is {mode}")


cli.close()
