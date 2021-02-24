# **Poucet — SX1280 LoRa 2.4 GHz Ranging Transceivers source files**

## **Project Brief**
This project contains the source files for LoRa 2.4 GHz ranging boards.
The ranging boards are based on Semtech SX1280 ranging transceivers monitored by a STM32 Nucleo, either Nucleo L432KC or Nucleo L476RG.
The ranging boards can estimate continuously their mutual distances through time-of-flight measurements , and can be used to mount a localization system.
The typical use case is to designated one of the boards as Master and the others as Slaves. The Master will proceed to perform rangings with the slaves one after each other. Then ,  the position of the Master board can be estimated with a multilateration process.

For more information on Semtech Sx1280 and the ranging, check the following URL:
https://www.semtech.com/products/wireless-rf/24-ghz-transceivers/sx1280

For more information on STM32 Nucleo boards, please check:
https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html


## **Getting set up**
Installing and running the project should take only a few minutes !
* Instructions
    0- You will need git on your computer. If it not already done, install git: https://github.com/git-guides/install-git 
    1- Git clone this repository
    1- This project is based on Mbed Client 1 for compilation management. Install Mbed client 1 as described at https://os.mbed.com/docs/mbed-os/v6.7/build-tools/mbed-cli-1.html.
    * You need Python 3.7x. Can be downloaded at https://www.python.org/downloads/. 
    Do not forget to add the Python installation directory and the Scripts subdirectory (which contains pip) to your path. You should be able to run Python and Pip in a terminal after that.
    * You may want to run this project within a virtual environment. In a virtual environment, the libraries and python interpreter that you are using exist only within the virtual environment and not on your whole system. This allows using different versions of the same library and different interpreters in each virtual environment you have.
    To create a virtual environment, go to the directory containing the root of this project and run:
    '''
    python -m venv Poucet_LCIS_LoRa24
    '''
    * You can then install Mbed client 1 with the following command:
    ```console
    pip install mbed-cli
    ```
    Check that Mbed client 1 is working by prompting:
    ```console
    mbed config
    ```  
    You should see Mbed configuration being displayed if everything is working fine.
    1- Check your installation — at that point everything should be ready. Plug two ranging boards (it does not amtter if they are based on Nucleo L432KC or L476RG — the script will automatically take care of recognizing the model and deploying the proper firmware) to your computer and run:
    ```console
    cd Lora24_firmware
    python Deployment.py
    ```
    This should take a little while as this is your first compilation and Mbed client has to build all the drivers. The drivers are built as a static library, hence, this step will be only done once unless you clean or modify the drivers. Thus, your next compilations should only take a few seconds.
    Once the command is completed, open the serial port of the ranging board that has been granted the lowest driver letter on your computer.
    This board has been automatically designated as the ranging Master and should be conintously estimating the distances between itself and all the other baords you plugged.
    You should receive ranging reports like this one:
    ```json
    {"type":"LoRa2.4","frequency":"2.402 GHz","bw":"BW 1.6 MHz","sf":"SF8","cr":"CR 4/8","rssi":"-69","zn":"0","distance":"8.8296","uncertainty":"3.7063","fei":"26245","initiator":"master","target":"1"}
    ```
    You're all set !
    *Note:* Currently, the deployment tool is only supported on Windows. More detailed instructions on  how to use the deployment tool and the firmware can be found below, in *Project Manual* section.

## *Project Description**
The firmware is based on Arm Mbed OS 6 bare-metal profile. As a consequence the built Nucleo binaries are lightweight (about 80 ko) and building the project is significantly faster comapred to a typical arm mbed project. If you want to swtich, check *Switching from Arm Mbed OS 6 bare-metal profile to the full RTOS* in the *Project Manual* section.
The SX1280 driver provided was originally based on Arm Mbed 5 

This project contains:
* Ranging boards firmware: several programs are available. You can also easily create your own programs and use them with the deployment tool, which will be much master than compiling and flashing each board manually.
Each program dependencies and source files are defined in json profile file, which are found at the root of the Lora24_firmware directory.
The default profile is currently Poucet.json. If you want to compile to deploy you new project MyProject.json, you can simply use the flag *-build* for the Deployment script:
```console
python Deployment.py -build MyProject.json
```
For further instructions on the content of these json profiles, refer to the *Project Manual* section.
* Deployment tool: a bunch of useful functions for compilation and deployment automation are featured in the Deployment python module. These functions are detailed in *Project Manual*, *Deploying your firmware* subsection.
* Calibration and characterization scripts, in the *Calibration* subdirectory: allow a utomatically calibrating and charactzering the performances of the ranging boardfs in different scenarios. Time-of-Flights measurements are very sensitive to various sources of delays introduced by manufacturing inconsistencies, including notably the antenna path delay and the crystal clock drift. These parameters typically differ from one chip to another, and need to be individually estimated to calibrate the distance estimations. The calibration scripts will automatically estimate these parameters for you if you plug a board requiring calibration to your computer.  
* MQTT bridge, in *Bridge* subdirectory: a *bridge* is a python script that collects the data on the serial port and publish them to MQTT topics. MQTT is an Iot messaging protocol; if you are familiar with MQTT, you can found information at:
https://mqtt.org/
The bridge is mainly intended to send the distance estimations (and other physical parameters) to an Positioning System that performs the multilateration process. This project is compatible with SecureLoc Localization Engiun, as explained below.
* SecureLoc middleware, in *SecureLoc* subdirectory: SecureLoc is an open source localization engine tailored for security applications, featuring 3D rendering based on Panda 3D. 
You can use SecureLoc to mount a localization system with the LoRa 2.4 Ghz ranging boards.
To install SecureLoc, run:
```console
git clone https://github.com/Hedwyn/SecureLoc.git
```
And follow the instructions in README @https://github.com/Hedwyn/SecureLoc
The subdirectory contains:
* A MQTT bridge to convert the serial port output to SecureLoc format.
**/!\ Note:** You need compile the poucet firmware with the following macro in main.cpp to make it work:
```c
#define SECURELOC
```
* A GPS coordinates converting script to convert the local coordinates computed by SecureLoc to global GPS coordinates

The characteriztion scripts will automatically collect the distance estimations and plot them against your chosen parameters for you. They are well-suited for measurement campaigns.

## **Project Manual**
### Selecting and creating programs
The deployment tool provides fine control of each program source files: when using the deployment script, Mbed will **not** pick automatically all the subfolders in *LoRa24_firmware*.
Instead, each program has a json profile defining the source files and dependencies. For example, Poucet.json contains:
```json
{"File Description": [
    "Contains the build configuration for this project.",
     "The list of directories to include during the compilation process is given in the field name ProjectLibs.",
     "The OS as well as the libraries provided in Drivers will be built as a static library and linked during project compilation.",
     "Regular compilations will NOT rebuilt the libraries given in Drivers. if you want to re-build the drivers, please proceed to a clean build (See Documentation) "
    ]
}
{"OS": "mbed-os", "Drivers":["SX1280Lib"], "ProjectLibs": ["main", "Peripherals", "Utils"], "Default build profile": "release"}
```
The file description entry is optional and only intended for documentation purposes. The second entry is mandatory and contains your project configuration:
* **"OS"** contains the name of the directory that contains Mbed OS. By default, mbed client installs it in *mbed-os*, but if you ever need to rename that folder you should modify the **"OS** field accordingly.
* **"Drivers"** contains a list of all the required external drivers that are not natively part of Mbed OS. For this project, you will most likely only need the SX1280 driver. It is installed by default in the *SX1280Lib* folder. If you ever need to rename that folder, change the driver name accordingly in the list contained in the *"Drivers"* field.
* **"ProjectLibs"** contains the list of the source files and folders for your project. All the files contained in each folder provided will be included in your project. One of them should obviously include a *main*. You should append all the scripts that you wrote for your program within that list. It is more convient to encapsulate your files into one or several folders and append these folders to the list rather than adding each file one by one. 
* **"Default build profile"** specifies which mbed build profile should be used by default when building your program. Mbed profiles contains a set of approriate compilation flags and options for a given use case (e.g., Release or Debug). Some information on the content and the principles of Mbed build profiles can be found here: https://os.mbed.com/docs/mbed-os/v6.7/build-tools/build-profiles.html. Note that you can override the default build profile by using the *-profile* flag when calling the Deployment tool, e.g.:
```console
python Deployment.py -build MyProgram.json -profile debug
```


### Switching from Arm Mbed OS 6 bare-metal profile to the full RTOS
The bare-metal profile is minimalist and contains only the necessary drivers. If you ever need features that are not included in the bare-metal profile (e.g., RTOS functionalities), you can switch to the regular OS profile. To do that, edit mbed_app.json at the root of the project, and remove *"bare-metal"* from the list in *"requires"*:
```json
"requires":["bare-metal","drivers", "platform"],
```
Note that you do not need to download any additional driver to use the full RTOS. The source files of the whole OS are already there, they are simply not built when using the bare-metal profile.

### Updating Mbed-os 
If the version of Arm Mbed Os included in this project is not the latest when cloning, you can easily update it yourself. Mbed-os git repository can be found at https://github.com/ARMmbed/mbed-os.
Mbed-os is defined as a submodule of this repository. You can get the latest release by prompting at the project root:
```console
git submodule update --init Lora24_firmware/mbed-os
```

## Configuring the physical layer
Ranging performances highly depend on the parameters of the LoRa 2.4 GHz radio physical layer.
This includes notably:
* Spreading Factor: the Spreading Factor (SF) defines the length of a single LoRa symbol. The higher the spreading factor, the lower the ranging rate. Increasing the SF factor will improve the sensitivity of the reception and make the communications more robust to interferences and multipath effects. As a consequence, it will have a significant impact of the maximum communication range. The ranging mode of SX1280 supports SF from 5 to 10. 
* Bandwidth: LoRa modulation is based on Chirp Spread Spectrum (CSS). The bandwidth, which is the difference between the maximum and minimum frequency reached by the chirp, can be set to 400, 800 or 1600 Khz. It is recommanded to use the maximum bandwidth as it will greatly increase the ranging accuracy.
* Channel: 40 frequency channels are available on the SX1280. It is recomanded to estimate the distances over multiple channels rather than a single one to improve the ranging process accuracy.
All physical parameters can be set easily in the Poucet firmware source files (see main.h in *main* subdirectory).

## Contact
*Project Maintainer:* [Baptiste Pestourie](mailto:baptiste.pestourie@lcis.grenoble-inp.fr?subject=[GitHub]Poucet LoRa 2.4 Ghz)
baptiste.pestourie@lcis.grenoble-inp.fr 

*Project Manager:* [Denis Genon-Catalot](mailto:denis.genon-catalot@lcis.grenoble-inp.fr?subject=[GitHub]Poucet LoRa 2.4 Ghz)
denis.genon-catalot@lcis.grenoble-inp.fr







 



    
