import subprocess
import shutil
from enum import Enum
import os, string
import sys

class Os(Enum):
    WIN = 0
    LINUX = 1
    MAC = 2

local_os = Os.WIN

DEVICE_MODEL = 'NODE_L432KC'
MAKE_CALL = "make all BIN="
BIN_EXTENSION = ".bin"
MASTER_BIN_PREFIX = "Master"
SLAVE_BIN_PREFIX = "Slave"
MAX_SLAVES_NUMBER = 6
MAKEFILE_FLAG = "MAIN_FLAGS="
MASTER_CONSTANT_NAME = "DEVICE_MASTER"
SLAVE_CONSTANT_NAME = "DEVICE_SLAVE"


def call_cmd(cmd):
    p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr= subprocess.STDOUT, shell = True)
    for line in iter(p.stdout.readline, b''):
        # displaying the command's output in stdout. Automatic new line in Python' print is removed
        print(line.decode('utf-8'), end = "")

def compile(bin_name, autogen_flags = True, flags = ""):
    if autogen_flags:
        flags += " " +  MAKEFILE_FLAG + '"'
        # If the device is master, defining the MASTER constant and undefining all the slaves constants
        if MASTER_BIN_PREFIX in bin_name:
            flags += ("-D" + MASTER_CONSTANT_NAME + " ")
            for idx in range(1, MAX_SLAVES_NUMBER):
                flags += "-U" + SLAVE_CONSTANT_NAME + " "
        # If the device is slave, defining the corresponding slave ID and undefining the others
        if SLAVE_BIN_PREFIX in bin_name:
            flags += ("-U" + MASTER_CONSTANT_NAME + " ")
            for idx in range(1, MAX_SLAVES_NUMBER):
                if idx == int(bin_name.split(".")[0][-1]):
                    flags += ("-D" + SLAVE_CONSTANT_NAME + str(idx) + " ")
                else:
                    flags += ("-U" + SLAVE_CONSTANT_NAME + str(idx) + " ")   
        cmd = MAKE_CALL + bin_name + " " + flags
        call_cmd(cmd)


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
          

def flash_device(device_path, device_firmware):
    # to flash the STM32, the device must be mounted and the binary at the root of its local memory
    bin_path = device_firmware + BIN_EXTENSION
    if os.path.exists(device_path):
        try:
            os.system("copy " + bin_path + " " + device_path + "\\" + bin_path)
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


def is_drive_stm32(drive, device_model = DEVICE_MODEL):
    if (local_os != Os.WIN):
        print("The drive name can only be checked on Windows")
    else:  
        drive_info = subprocess.check_output(["cmd","/c vol "+ drive])
        # decoding drive information to utf-8- some characters may be dropped 
        drive_info_utf8 = ""
        for char in drive_info:
            decoded_char =  chr(char)
            drive_info_utf8 += decoded_char
        return(device_model in drive_info_utf8)


def get_drives_win(device_model = DEVICE_MODEL):
    if (local_os != Os.WIN):
        print("The drives can only be detected on Windows")
    else:
        drives = ['%s:' % d for d in string.ascii_uppercase if os.path.exists('%s:' % d)]
        stm32_list = [drive for drive in drives if is_drive_stm32(drive)]
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
        drive_to_bin_dic[stm32] = bin_name
    return(drive_to_bin_dic)

def deployment(ordering_list = None):
    drive_to_bin_dic = associate_bins_to_drives(ordering_list)
    devices_path = []
    bin_names = []
    total_slaves = len(drive_to_bin_dic) - 1
    slaves_number_flag = "TOTAL_SLAVES=" + str(total_slaves)
    for drive in drive_to_bin_dic:
        bin_name = drive_to_bin_dic[drive]
        compile(bin_name, flags = slaves_number_flag)
        devices_path.append(drive)
        bin_names.append(bin_name)
        clean_objs()
    print(devices_path)
    flash_all_devices(devices_path, bin_names)

def n_compile(n):
    for i in range(n):
        compile("Slave" + str(i + 1), "TOTAL_SLAVES=" + str(n))

if __name__ == "__main__":
    argc = len(sys.argv)
    ordering_list = None
    no_compile = False
    if argc > 1:
        for arg in sys.argv[1:]:
            # checking flags
            if arg[0] == "-":
                flag = arg[1]
                if (flag == 'o'):
                    order_flag = True
                    if len(arg) == 2:
                        print("You must provide the ordered list of drives you want to flash after the -o flag")
                    else:
                        ordering_list = arg[2:]
                elif (flag == 'f'):
                    print("Flashing only")
                    no_compile = True
                    flash_all_devices()
                elif (flag == 'h'):
                    print("Usage: -o or -order to specify the order of drives to flash")

    # launching deployment
    if not(no_compile):
        deployment(ordering_list)




