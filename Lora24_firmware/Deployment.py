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
@date 2018 February 1st
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

class Os(Enum):
    WIN = 0
    LINUX = 1
    MAC = 2

local_os = Os.WIN

DEVICE_MODEL = 'NODE_L432KC'
DEVICE_MODELS = ['L432KC', 'L476RG', 'L073RZ']
MAKE_CALL = "make all BIN="
BIN_EXTENSION = ".bin"
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
TARGETS = ["NUCLEO_L432KC", "NUCLEO_L476RG", "NUCLEO_L073RZ"]
TOOLCHAIN = "GCC_ARM"
PROJECT_NAME = "SX1280"
APP_CONFIG_FILE = "mbed_app.json"
VERBOSE = True

class ProjectConfig:
    def __init__(self, os_path, drivers_path, project_libs, profile):
        self.os_path = os_path
        self.drivers_path = drivers_path
        self.project_libs = project_libs
        self.profile = profile

def parse_build_config(build_config_path = BUILD_CONFIG_PATH):
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
    
    cmd += " --library"

    if config.profile:
        cmd += " --profile " + config.profile
    if clean:
        cmd += " -c"      
    print(cmd)
    call_cmd(cmd)

def compile(id, total_slaves, config, target = TARGETS[0], flash = False, clean =  False):
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

    # setting the target
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

    # copying compiled binary to the firmware directory
    
    copyfile("BUILD/" + target + "/" + release_name + "/" + bin_name + ".bin", FIRMWARE_DIR + "/" + bin_name + ".bin" )
    # try:
    #     copyfile("BUILD/" + release_name + "/" + PROJECT_NAME + ".bin", FIRMWARE_DIR + "/" + bin_name )
    # except:
    #     print("Could not copy binary into the firmware directory. Compilation may have failed")


def clean_objs():
    call_cmd("make clean_objs")

def clean():
    call_cmd("make clean")

def gen_bin_names(total_devices = 1):
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
            flash_device(device_path, bin_name)


def is_drive_stm32(drive, device_models = DEVICE_MODELS):
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
        stm32_list = [drive for drive in drives if is_drive_stm32(drive, device_models)]
        return(stm32_list)



def associate_bins_to_drives(ordering_list = None):
    if (local_os != Os.WIN):
        print("The drives can only be detected on Windows")
        return
    drive_to_bin_dic = {}
    if ordering_list:
        devices_number = len(ordering_list)
        stm32_list = [drive_letter + ":" for drive_letter in ordering_list]
        bin_names = gen_bin_names(devices_number)    
    else:
        stm32_list = get_drives_win()
        devices_number = len(stm32_list)
        bin_names = gen_bin_names(devices_number)

    for bin_name, stm32 in zip(bin_names, stm32_list):
        target = is_drive_stm32(stm32)
        print(target)
        drive_to_bin_dic[stm32] = bin_name, target
    return(drive_to_bin_dic)

def deploy(build_config_path = BUILD_CONFIG_PATH, ordering_list = None, total_slaves = None, clean_drivers = False):
    drive_to_bin_dic = associate_bins_to_drives(ordering_list)
    devices_path = [drive for drive in drive_to_bin_dic]
    bin_names = [drive_to_bin_dic[drive][0] + BIN_EXTENSION for drive in drive_to_bin_dic]
    targets = [drive_to_bin_dic[drive][1] for drive in drive_to_bin_dic]
    total_devices = len(devices_path)
    project_conf = parse_build_config(build_config_path)

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



        

if __name__ == "__main__":
    argc = len(sys.argv)

    # default arguments 
    total_slaves = None
    build_config_path = BUILD_CONFIG_PATH
    clean = False

    for idx, argv in enumerate(sys.argv[1:]):
        if argv[0] == "-":
            # flag detected
            if argv[1:] == "build":
                build_config_path = sys.argv[idx + 2]

            if argv[1:] == "rebuild":
                build_config_path = sys.argv[idx + 2]
                clean = True

            elif argv[1:] == "total_slaves":
                try:
                    total_slaves = int(argv[idx + 2])
                except:
                    print("Please provided an integer value for the total number of slaves")
         
    deploy(build_config_path = build_config_path, total_slaves =  total_slaves, clean_drivers = clean)





