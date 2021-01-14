import serial

COM_PORT = 'COM5'

def open_serial(com_port):
    try:
        port = Serial(com_port, 115200)
    except:
        print("Serial port not available")
        raise()
    return(port)

def read_serial(port):
    exit_flag = False
    while (not(exit_flag)):
        line = port.readline().decode('utf-8')
        




