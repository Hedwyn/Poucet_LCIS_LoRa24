from serial import *

# def readSerial(port):


#     try:
#         while (True):     
#             line = port.read()
#             print(line.decode('utf-8'))
#             # if not(no_return):
#             #     print(line)
#             #     no_return = True            
#             #     line = ""
#             # while (port.in_waiting > 0) and no_return:
#             #     try:
#             #         received_char = port.read().decode('utf-8')
#             #         line += received_char
#             #         print(received_char)
#             #         if received_char == '\n':
#             #             no_return = False
#             #     except:
#             #         print("Formatting failed")
#             #         pass
                    

#     except KeyboardInterrupt:
#         print("CTRL + C receivefd, exitting")
    
# def openPort(path):
#     try:
#         port = Serial(path)
#         return(port)
#     except:
#         print(path + "not found, make sure to plug the device")


# def run(path):
#     port = openPort(path)
#     if port:
#         readSerial(port)

if __name__=="__main__":
    SERIALPATH = 'COM10'
    # run(SERIALPATH)
    port = Serial('COM10', baudrate = 115200)
    while True:
        if port.in_waiting > 0:
            line = port.readline().decode('utf-8')
            print(line)