"""****************************************************************************
Copyright (C) 2019 LCIS Laboratory - Baptiste Pestourie

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
This program is part of the SecureLoc Project @https://github.com/Hedwyn/SecureLoc
 ****************************************************************************

@file Deployment.py
@author Baptiste Pestourie
@date 2019 March 1st
@brief Compiles a single source file multiple times, 
generates unique IDs for each, sends the hex files via ftp and triggers Decawinos flashing on the RPIs"
@see https://github.com/Hedwyn/SecureLoc
"""

# Cleans the directory, compiles anchor code and generates a different hex for each ID.
import subprocess
from subprocess import Popen, PIPE, STDOUT
import os
import fabric
import paramiko
import spur
import time
from pythonping import ping
# from Make import *

MY_IP = '169.254.152.228'
SHELL_ON = True
BOOTLOADER = "teensy_loader_cli"
HOSTS_LIST = ['Rasp1','Rasp2','Rasp3','Rasp4']
USERNAMES = {'Rasp1':'pi','Rasp2':'pi','Rasp3':'pi','Rasp4':'pi'}
USERS_PWD = {'Rasp1':'raspberry','Rasp2':'raspberry','Rasp3':'raspberry','Rasp4':'raspberry'}
HOSTPATH = '/home/pi/'
CONFIG_PATH = 'Config'
DEFAULT_CONF = 'config.txt'

DEFAULT_PROJECT_NAME = 'Anchor'
hex_name = 'node'

DEFAULT_NB_ANCHORS = 4
DEFAULT_ID = '1'

HOSTS_CONFIG = 'hostnames.txt'
PROJECTS_DIR = 'Projects'
BIN_DIR = 'bin'
ID_LENGTH = 1
anchors_table = {}
OPENOCD_SUCCESS_MSG = "Verified OK" ## may change with other/newer versions of OpenOCD

class Console():
	_console = None
	@property
	def console(self):
		return Console._console
	@console.setter
	def console(self,p):
		Console._console = p

	def shell_exec(self,cmd):
		"""executes the given command in a shell.
		Can display the output into the compilation menu console if passed in argument."""
		#subprocess.run(cmd, SHELL_ON)
		if self.console == None:

			p = Popen(cmd,shell = SHELL_ON)
			p.wait()
		else:
			self.console.console_handler(cmd)
	def print(self, msg):
		"""prints the message in the current console. Defaults to the current shell"""
		if self.console == None:
			print(msg)
		else:
			self.console.console_display(msg)

console = Console()

def read_config(configname = DEFAULT_CONF):
	"""reads and extracts the data from config file, i.e. Raspberry host and user names,
	anchors names, and raspberry/anchors association"""
	hosts = []
	with open(HOSTS_CONFIG) as f:
		for i, line in enumerate(f):
			if not(line.startswith('#')):
				data = line.split()
				if len(line) < 3:
					print("Error at line " + str(i) + " in" + HOSTS_CONFIG)
					continue
				credentials = {}
				credentials['hostname'] = data[0]
				credentials['username'] = data[1]
				credentials['password'] = data[2]
				hosts.append(credentials)
	return(hosts)

def check_connected_hosts(hostslist):
	connected_hosts = []
	for host in hostslist:
		responses = ping(host["hostname"], timeout = 0.3, count = 1)
		for r in responses:
			if r.message:
				connected_hosts.append(host)
	return(connected_hosts)

def shell_exec(cmd):
	"""executes the given command in a shell.
	Can display the output into the compilation menu console if passed in argument."""
	#subprocess.run(cmd, SHELL_ON)

	p = Popen(cmd,shell = SHELL_ON)
	p.wait()


def send_hex_file(file_path, destination,password,path, project_name):
	"""sends hex file to raspberry with the given ip address"""
	console.shell_exec("pscp -scp -pw " + password + " " + file_path + " " + destination + ":" + path)


def deploy_hex_files(config = DEFAULT_CONF, project_name = DEFAULT_PROJECT_NAME):
	"""triggers the compilation and deploys the hex files on the raspberry hosts"""

	# getting config
	[hosts_list,usernames,passwords,anchors_names] = read_config(config)
	nb_anchors = len(anchors_names)

	# cleaning previous local hex files
	clean(project_name)

	# compiling main file, generating an hex file with unique ID for each anchor
	compilation(nb_anchors, project_name)
	for host in hosts_list:
		# flushing previous hex files
		local_clean(host, usernames[host],passwords[host])

	for host in hosts_list:
		for anchor in anchors_table[host]:
			filename = hex_name + anchor + ".hex"
			destination = usernames[host] + "@" + host
			path = HOSTPATH
			pwd = passwords[host]
			send_hex_file(filename, destination,pwd,path,project_name)


def local_flash(hostname,username,password):
	"""triggers device flashing on the given rasp host.
	Note: for OpenOCD reset config use : 'reset_config srst_only connect_assert_srst"""
	#starting ssh client

	with paramiko.SSHClient() as ssh:
	#ssh.connect(hostname,username = username,password = password)
		ssh.load_system_host_keys()
		ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		ssh.connect(hostname,port =22,username = username, password = password)

		# cleaning previous hex files
		(stdin, stdout, stderr) = ssh.exec_command('ls')
		# flashing the device
		(stdin, stdout, stderr) = ssh.exec_command('nohup sudo openocd -f /home/pi/openocd/openocd.cfg')
		# (stdin, stdout, stderr) = ssh.exec_command('nohup echo hey')
		stdout.channel.recv_exit_status()
		# checking if the device was flashed successfully
		success = False

		output = ""
		for line in stderr.readlines():
			if OPENOCD_SUCCESS_MSG in line:
				success = True
			output += line
		if success:
			print("Device flashed successfully")
		else:
			print(output)
			print("\n /!\ Device flash failed ! /!\ \n")

def start_remote_clients(hostslist):
	for host in hostslist:
		with paramiko.SSHClient() as ssh:
			ssh.load_system_host_keys()
			ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
			ssh.connect(host["hostname"],port =22, username = host["username"], password = host["password"])	
			transport = ssh.get_transport()
			channel = transport.open_session()
			channel.exec_command('nohup sudo pkill python')
			channel.recv_exit_status()
			channel = transport.open_session()
			channel.exec_command('nohup python3 /home/pi/client.py 169.254.159.156 > /dev/null 2>&1')# + ' > /dev/null 2>&1')

def stop_remote_clients(hostslist):
	for host in hostslist:
		with paramiko.SSHClient() as ssh:
			ssh.load_system_host_keys()
			ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
			ssh.connect(host["hostname"],port =22, username = host["username"], password = host["password"])	
			transport = ssh.get_transport()
			channel = transport.open_session()
			channel.exec_command('nohup sudo pkill python')
			channel.recv_exit_status()

def local_clean(hostname,username,password):
	"""triggers anchor flashing on the given rasp host"""
	#starting ssh client

	#with paramiko.SSHClient() as ssh:
	#ssh.connect(hostname,username = username,password = password)
	ssh = paramiko.SSHClient()
	ssh.load_system_host_keys()
	ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
	console.print(hostname + " " + username)
	ssh.connect(hostname = hostname,port =22,username = username, password = password,timeout = 3)


	# cleaning previous hex files
	(stdin, stdout, stderr) = ssh.exec_command('ls')

	# flushing the anchors in anchor_list

	(stdin, stdout, stderr) = ssh.exec_command('rm  ~/Desktop/*.hex')
	stdout.channel.recv_exit_status()
	ssh.close()

		

if __name__ == "__main__":
	# send_hex_file('BUILD/IMST282A/GCC_ARM-RELEASE/Master.elf', 'pi@raspberrypi.local', 'lcis', 'Master.elf', HOSTPATH)
	# send_hex_file('openocd.cfg', 'pi@raspberrypi.local', 'lcis', 'openocd.cfg', HOSTPATH)
	# send_hex_file('../Bridge/bridge_pizero_imst.py', 'pi@raspberrypi.local', 'lcis', 'client.py', HOSTPATH)
	# local_flash('raspberrypi.local', 'pi', 'lcis')
	# hosts= read_config()
	# print(check_connected_hosts(hosts))
	stop_remote_clients(check_connected_hosts(read_config()))
