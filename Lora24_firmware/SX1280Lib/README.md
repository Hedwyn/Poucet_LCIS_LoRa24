# SX1280Lib

## Project Brief
SX1280 driver for Arm Mbed OS 6.
Semtech's driver original source code can be found at https://os.mbed.com/teams/Semtech/code/SX1280Lib/
The original driver has not been updated since 2018, and was based on Arm Mbed OS 5 at that time.
Arm Mbed OS has upgraded to version 6 in 2020. As Arm Mbed OS does not provide backward comptability, the original driver was not supported within Mbed OS 6 as it.
This repo brings the following modifications to SX280 driver:
* The driver has been rewritten Arm Mbed OS 6. Note that i will *not* be supported by previous versions (2 and 5).
* Several missing functionalities (mentioned in SX1280 datasheet) have been implemented
* A non-exhaustive list of minor bugs and discrepancies have been fixed

 *Disclaimer* This repo is intended for research pruposes and is by no means approved or maintained by Semtech. Original driver is under BSD license.

