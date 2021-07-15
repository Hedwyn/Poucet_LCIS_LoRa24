"""****************************************************************************
Copyright (C) 2019 Project Poucet - LCIS Laboratory
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
@file application.py
@author Baptiste Pestourie
@date 2020 December 1st
@brief Deployment module - handles complation & deployment automation fore SX1280-based ranging boards
@see https://github.com/Hedwyn/Poucet_LCIS_LoRa24 {private repo- you need permission access}
"""

import subprocess
import shutil
from enum import Enum
import os, string
import sys
import json
from shutil import copyfile, rmtree
import time
import argparse 
import RemoteFlash

class Os(Enum):
    WIN = 0
    LINUX = 1
    MAC = 2

local_os = Os.WIN

DEVICE_MODEL = 'NODE_L432KC'
DEVICE_MODELS = ['L432KC', 'L476RG', 'L152RE']
MAKE_CALL = "make all BIN="
BIN_EXTENSION = ".bin"
ELF_EXTENSION = ".elf"
MASTER_BIN_PREFIX = "Master"
SLAVE_BIN_PREFIX = "Slave"
MAX_SLAVES_NUMBER = 10
MAKEFILE_FLAG = "MAIN_FLAGS="
MASTER_CONSTANT_NAME = "DEVICE_MASTER"
SLAVE_CONSTANT_NAME = "DEVICE_SLAVE"

MBED_COMPILE = 'mbed compile'
LIBS = ['SX1280Lib', 'mbed-os']
LIBS_PATH = 'BUILD/'
BUILD_CONFIG_PATH = 'Poucet.json'

DEVICE_ID_MACRO_NAME = "DEVICE_ID"
TOTAL_SLAVES_MACRO_NAME = "TOTAL_SLAVES" 
FIRMWARE_DIR = "Firmware"
TARGET = "NUCLEO_L432KC"
TARGETS = ["NUCLEO_L432KC", "NUCLEO_L476RG", "NUCLEO_L152RE"]
TOOLCHAIN = "GCC_ARM"
PROJECT_NAME = "SX1280"
APP_CONFIG_FILE = "mbed_app.json"
VERBOSE = True
HOST_FILE = "hostnames.json"
DEFAULT_ELF_NAME = "imst_firmware.elf"

class ProjectConfig:
    """Contains all the paramaters and flags required to build a given project"""
    def __init__(self, os_path, drivers_path, project_libs, profile):
        self.os_path = os_path
        self.drivers_path = drivers_path
        self.project_libs = project_libs
        self.profile = profile

def parse_build_config(build_config_path = BUILD_CONFIG_PATH):
    """Extract the data from the configuration file.
    Parameters
    ----------
    build_config_path: str
        path to the build configuration profile.
    Returns: ProjectConfig
    -------
    A ProjectConfig instance containing all the extracted data"""
    with open(build_config_path) as f:
        for line in f:
            if line != '\n':
                try:
                    log = json.loads(line)
                    print(log["Drivers"])
                except:
                    log = None                 
                # if log and ["Drivers", "ProjectLibs", "OS", "Default build profile"] in log:
                if log and all(key in log for key in ["Drivers", "ProjectLibs", "OS", "Default build profile"]):
                    print("Build configuration file successfully parsed")
                    return(ProjectConfig(log["OS"], log["Drivers"], log["ProjectLibs"], log["Default build profile"]))
        print("Could not find a proper json configuration. Check build file formatting")

def call_cmd(cmd):
    """Launches a subprocess with shell command.
    Parameters
    ----------
    cmd: str    
        The command to pass to the shell"""
    p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr= subprocess.STDOUT, shell = True)
    for line in iter(p.stdout.readline, b''):
        # displaying the command's output in stdout. Automatic new line in Python' print is removed
        print(line.decode('utf-8'), end = "")

def import_app_config(os_path):
    try:
        copyfile(APP_CONFIG_FILE, os_path + "/" + APP_CONFIG_FILE)
    except:
        print("App config file not found - make sure to define an mbed config file at the root of this project")


def clean_drivers_build(config):
    # removing previous static library. Mbed copile clean flag is flawed when building libraries so this step has to be done by hand
    root_dir = os.path.basename(os.getcwd())
    build_path = 'BUILD/libraries/' + root_dir
    if os.path.exists(build_path):
        build_path = 'BUILD/libraries/' + root_dir
        print("Cleaning previous build...")
        rmtree(build_path)

def build_drivers(config, clean = False, target = TARGET):
    """Builds the drivers as static libraries for a given project. 
    The names of the drivers are defines in the project json file.
    Parameters
    ----------
    config: ProjectConfig
        Configuration of the project to compile for
    clean: bool
        whether to rebuild all the source files or not
    target: str
        device model to compile for
    """
    root_dir = os.path.basename(os.getcwd())
    static_lib_path = 'BUILD/libraries/' + root_dir + '/' + target + '/GCC_ARM-' + config.profile.upper() + '/' + 'libmbed-os.a'
    print(static_lib_path)
    # checking that the OS has an app config file. If not, importing it.
    if not(os.path.exists(config.os_path + "/" + APP_CONFIG_FILE)):
        print("Importing app config file...")
        import_app_config(config.os_path)
    else:
        print("App config file found")
    cmd = MBED_COMPILE
    cmd += " --target " + target
    cmd += " --source " + config.os_path
    for driver in config.drivers_path:
        cmd += " --source " + driver

    # assigning custom targets dir
    cmd += " --custom-targets ."  

    cmd += " --library"
    if config.profile:
        cmd += " --profile " + config.profile
    if clean:
        cmd += " -c"      
    print(cmd)
    call_cmd(cmd)

def compile(id, total_slaves, config, target = TARGETS[0], flash = False, clean =  False):
    """Compiles a given project with Arm-Mbed client 1. 
    Defines several macros related to the project, the target, and the project dependencies.
    Parameters
    ----------
    total_slaves: int
        The total number of slaves that will be used on the platform.
    config: ProjectConfig
        An instance of a project configuration object.
    target: str
        Device model to compile for
    flash: bool
        whether the device should be flashed after compilation
    clean: bool
        whether to rebuild the project static libraries"""

    if total_slaves == 0:
        total_slaves = 1
    cmd = MBED_COMPILE + ' '
    release_name = (TOOLCHAIN + "-" + config.profile.upper())  if config.profile else TOOLCHAIN
    root_dir = os.path.basename(os.getcwd())
    static_libs_path = "BUILD/" + "libraries" + "/" + root_dir+ "/" + target + "/" + release_name
    print(static_libs_path)
    if total_slaves == 0:
        total_slaves = 1
    if clean:
        cmd += "-c"
    for lib in config.project_libs:
        cmd += " --source " + lib
    
    if os.path.exists(static_libs_path):
        cmd += " --source " + static_libs_path 
    else:
        print("OS and drivers have not been built yet. Proceeding to build them.")
        build_drivers(config, clean = clean, target = target)
        # waiting for linker script generation
        time.sleep(2)

    # assigning custom targets dir
    cmd += " --custom-targets ."

    # setting the target
    if target:
        cmd += " --target " + target
        cmd += " -D" + target

    # defining the device ID and the total number of slaves
    cmd += " -D" + DEVICE_ID_MACRO_NAME + "=" + str(id) + " -D" + TOTAL_SLAVES_MACRO_NAME + "=" + str(total_slaves)


    # defining the binary name
    bin_name = "Master" if id == 0 else ("Slave" + str(id))
    cmd += " -N" + bin_name
    if config.profile:
        cmd += " --profile " + config.profile
  
    if flash:
        cmd += " --flash"
    print(cmd)
    call_cmd(cmd)

    # copying compiled binary (bin and elf) to the firmware directory
    
    copyfile("BUILD/" + target + "/" + release_name + "/" + bin_name + ".bin", FIRMWARE_DIR + "/" + bin_name + ".bin" )
    copyfile("BUILD/" + target + "/" + release_name + "/" + bin_name + ".elf", FIRMWARE_DIR + "/" + bin_name + ".elf" )

def gen_bin_names(total_devices = 1):
    """Generates the bin prefixes list.
    The first binary is defined as Master, the following ones as slave with a slave index"""
    bin_names = []
    if total_devices == 0:
        print("No binaries to generate as the total number of devices is 0")
    else:
        # First binary is always the Master
        bin_names.append(MASTER_BIN_PREFIX)
        for slave_index in range(total_devices - 1):
            # slaves are indexed starting to 1
            bin_names.append(SLAVE_BIN_PREFIX + str(slave_index + 1))
    return(bin_names)
          

def flash_device(device_path, bin_name):
    """Flashes a device locally or remotely.If the device is wired, the bin is copied to the device's drive.
    If the device is remote, the elf file is sent over through psctp and openocd is calle thourgh ssh.
    Parameters
    ----------
    device_path: str/tuple
        device's drive or credentials tuple for remote devices.
    bin_name:
        the binary to use
     """
    # to flash the STM32, the device must be mounted and the binary at the root of its local memory
    bin_path = FIRMWARE_DIR + "\\" + bin_name

    if os.path.exists(device_path):
        try:
            os.system("copy " + bin_path  + " " + device_path + "\\" + bin_name)
        except OSError:
            if not(os.path.exists(bin_path)):
                print("The binary could not be found. Make sure to generate the binaries before proceeding to flash")
        except SameFileError:
            print("Source and destiantion are the same file")
    else: 
        print("The drive could not be found")

def flash_all_devices(devices_paths = None, bin_names = None):
    """Flashes all the devices listed with a set of given binaries.
    Parameters
    ----------
    devices_path: list[str, tuple]
        The list of the device local path (for wired devices) or credentials tuple (for remote devices), in order.
    bin_names:
        The lists of binary files names, in order. Should be the same length as devices_path."""
    if not(devices_paths):
        print("Getting the paths to the devices currently connected as external drives")
        devices_paths = get_drives_win()
    if not(bin_names):
        print("Regenerating binary names..")
        bin_names = gen_bin_names(len(devices_paths))
    print(bin_names)
    
    if len(devices_paths) < len(bin_names):
        print("The number of paths provided is inferior to the number of binaries. Aborting")
    else:
        for bin_name, device_path in zip(bin_names, devices_paths):
            if isinstance(device_path, str):
                flash_device(device_path, bin_name + BIN_EXTENSION)
            else:
                host = dict(device_path)
                # sending the elf file
                bin_path = FIRMWARE_DIR + "\\" + bin_name
                RemoteFlash.send_hex_file(bin_path + ELF_EXTENSION,  host["username"] + "@" + host["hostname"], host["password"], DEFAULT_ELF_NAME, RemoteFlash.HOSTPATH)
                RemoteFlash.local_flash(host["hostname"], host["username"], host["password"])


def get_drive_model(drive, device_models = DEVICE_MODELS):
    """Checks the STM32 model of each connected drive. 
    Returns: str
    -------
    Device model or None if the model is unknown
    """
    if (local_os != Os.WIN):
        print("The drive name can only be checked on Windows")
    else:  
        drive_info = subprocess.check_output(["cmd","/c vol "+ drive])
        # decoding drive information to utf-8- some characters may be dropped 
        drive_info_utf8 = ""
        for char in drive_info:
            decoded_char =  chr(char)
            drive_info_utf8 += decoded_char
        for device in device_models:
            if device in drive_info_utf8:
                return("NUCLEO_" + device)
        return(False)


def get_drives_win(device_models = DEVICE_MODELS):
    if (local_os != Os.WIN):
        print("The drives can only be detected on Windows")
    else:
        drives = ['%s:' % d for d in string.ascii_uppercase if os.path.exists('%s:' % d)]
        stm32_list = [drive for drive in drives if get_drive_model(drive, device_models)]
        return(stm32_list)



def associate_bins_to_drives(ordering_list = None):
    if (local_os != Os.WIN):
        print("The drives can only be detected on Windows")
        return
    # parsing hosts
    hostslist = RemoteFlash.check_connected_hosts(RemoteFlash.read_config())

    drive_to_bin_dic = {}
    if ordering_list:
        devices_number = len(ordering_list) + len(hostslist)
        stm32_list = [drive_letter + ":" for drive_letter in ordering_list] + hostslist
        bin_names = gen_bin_names(devices_number)    
    else:
        stm32_list = get_drives_win() + hostslist
        devices_number = len(stm32_list) 
        bin_names = gen_bin_names(devices_number)

    for bin_name, stm32 in zip(bin_names, stm32_list):
        if isinstance(stm32, str):
            target = get_drive_model(stm32)
            key = stm32
        else:
            # remote targets are always IMST282A
            target = "IMST282A"
            key = tuple(sorted(stm32.items()))
        print(target)
        drive_to_bin_dic[key] = bin_name, target
    return(drive_to_bin_dic)

def deploy(project_conf, ordering_list = None, total_slaves = None, clean_drivers = False):
    # parsing USB drives
    drive_to_bin_dic = associate_bins_to_drives(ordering_list)
    devices_path = [drive for drive in drive_to_bin_dic]
    bin_names = [drive_to_bin_dic[drive][0] for drive in drive_to_bin_dic]

    # parsing hosts
    
    targets = [drive_to_bin_dic[drive][1] for drive in drive_to_bin_dic]
    total_devices = len(targets)

    if clean_drivers:
        clean_drivers_build(project_conf)
    print(bin_names, devices_path)
    # compiling firmware, one binary for each device
    n_compile(project_conf, total_devices, targets, total_slaves)
    

    # flashing them all
    flash_all_devices(devices_path, bin_names)


def n_compile(project_conf, n, targets = None, total_slaves = None):
    if not total_slaves:
        # substracting master (id 0) from the slaves count
        total_slaves = n - 1
    for i in range(n):
        if targets:
            compile(i, total_slaves, project_conf, target = targets[i])
        else:
            compile(i, total_slaves, project_conf)

def parse_remote_hosts():
    with open(HOST_FILE) as f:
        hosts = json.loads(f)
        for hostname in hosts:
            pwd = hosts[hostname]
            

def st_link_flash(conf_path, id = 0, total_slaves = 0, target = 'IMST282A'):
    conf = parse_build_config(conf_path)
    compile(id, total_slaves, conf, target, clean = True)


        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Manages and automates the compilation and deployment of LoRa 2.4 GHz Ranging transceivers firmware. By default, auto-detects the connected targetrs, then compiles and deploys the selected project firmware")
    parser.add_argument('project', help = "Builds the project defined in the json configuration file passed as argument.")
    parser.add_argument("-b", "--build", help = "Compiles without flashing. You should pass the total number of nodes to compile for as argument", type = int)
    parser.add_argument("-c", "--clean", help = "Cleans previous builds of the target project and related dependencies", action = "store_true")
    parser.add_argument("-t", "--target", help = "Cleans previous builds of the target project and related dependencies")

    args = parser.parse_args()

    if args.project:
        config = parse_build_config(args.project)
        if args.target:
            target = args.target
        else:
            target = TARGETS[0]
        if args.clean:
            clean_drivers_build(config)
        if args.build:
            targets = [target] * args.build
            n_compile(config, args.build, targets)
        else:
            deploy(config)
    else:
        print("No argument given. You should pass a project description json file as argument. Quitting")
        parser.print_help()

    # argc = len(sys.argv)

    # # default arguments 
    # total_slaves = None
    # build_config_path = BUILD_CONFIG_PATH
    # clean = False

    # for idx, argv in enumerate(sys.argv[1:]):
    #     if argv[0] == "-":
    #         # flag detected
    #         if argv[1:] == "build":
    #             build_config_path = sys.argv[idx + 2]

    #         if argv[1:] == "rebuild":
    #             build_config_path = sys.argv[idx + 2]
    #             clean = True
            
    #         if argv[1:] == "stlink":
    #             build_config_path = sys.argv[idx + 2]
    #             st_link_flash(build_config_path)
    #             sys.exit()


    #         elif argv[1:] == "total_slaves":
    #             try:
    #                 total_slaves = int(argv[idx + 2])
    #             except:
    #                 print("Please provided an integer value for the total number of slaves")
         
    # deploy(build_config_path = build_config_path, total_slaves =  total_slaves, clean_drivers = clean)





