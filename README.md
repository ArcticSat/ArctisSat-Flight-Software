# Flight Software for IrisSat
This repository contains software for the IrisSat Command and Data Handling (CDH) board. The directory structure is like so:
- /iris-fsw-libero contains the Libero project required for configuring the device.
- /iris-fsw-softconsole contains the SoftConsole project required to write and debug the software.


## Build Steps

### Required hardware
This project requires the Microsemi Smartfusion2 Maker Board.

### Required Software
This project is tested with SoftConsole 6.5 and Libero 12.2.

### Bootstrap
This script bootstraps the softconsole project, and It requires python 3 and arm-none-eabi toolchain (included in SoftConsole) to be installed. This script downloads the dependencies, and then builds libcsp which has a separate build step.
```
python ./scripts/bootstrap.py
```

To build libcsp with debug configuration, pass `--debug`
```
python ./scripts/bootstrap.py --debug
```

Note: The bootstrap script will attempt to apply a patch for LittleFS. If you have msys or mingw already installed then this should work fine. If you see an error you may have to manually patch, from the scripts dir:
```
patch -d ../iris-fsw-softconsole/Libraries/ -p2 < littlefs.patch
```

### Configure your workspace
1. Select the root folder (i.e. IrisSat-Flight-Software) as your SoftConsole workspace.
2. Go to **File -> Import**.
3. On the Import window, select **General -> Existing C/C++ Projects into Workspace**.
4. Add "iris-fsw-softconsole" to your workspace.

### Build
5. To build the SoftConsole project, go to **Project -> Build All**, or use **Ctrl-B**.

**Note**: The "Production" build of the softconsole project should be compiled before the Libero project is built because the Libero project will include the firmware binary as part of the FPGA image.

### Configure FPGA
6. Open the \*.prjx found in iris-fsw-libero, with Libero 12.1.
7. Connect the MSR board to the computer.
8. In the "Design Flow" panel on the left, double-click "Synthesize".
9. In the "Design Flow" panel on the left, double-click "Manage Constraints" and make sure  constraint\io\user.pdc is selected.
10. In the "Design Flow" panel on the left, double-click "Generate FPGA Array Data".
11. In the "Design Flow" panel on the left, double-click "Update eNVM Memory Content", double-click on the Data Storage client and select iris-fsw-softconsole/Production/iris-fsw-softconsole.hex as the content file.
12. In the "Design Flow" panel on the left, double-click "Run PROGRAM Action".

### Debug
13. Go to **Run -> Debug Configurations**.
14. Double-click **GDB OpenOCD Debugging** to create a new Debugging configuration.
15. Under the "Main" tab, browse and select the "iris-fsw-softconsole" project as the project.
16. Under the "Debugger" tab, change the Config options to: **"--command "set DEVICE M2S010" --file board/microsemi-cortex-m3.cfg"**
17. Under the "Startup" tab, make sure that "Pre-run/Restart reset" is not checked.
18. With the MSR board connected, click "Apply", and then "Debug" to run the software.

*Also if the Libero project is modified, the firmware should be regenerated and copied to the softconsole project. Note that the CoreSPI firmware must be generated from the Microsemi Firmware Catalog and copied to the firmware folder in the softconsole project*

## Software Dependancies:

### Cubesat Space Protocol (CSP)

This library is provides a communication protocol stack following the TCP/IP model. The MBSat flight software uses the CSP library for communication with the communication system on the satellite.



## Useful links:
1. Maker Board IoT Demo project: https://www.digikey.com/eewiki/display/microcontroller/Getting+Started+with+the+Microsemi+SmartFusion+2+Maker-Board
2. Maker Board "First Project" Demo: https://github.com/tstana/M2S010-MKR-KIT_FirstProj/wiki


## Software Update Instructions:

Remember to set the program flash spi pins to be enabled in flash freeze mode!
This can be done in the i/o editor. Make sure the mss_spi spi_0 module has the pins set as I/O not FPGA to ensue the option to change the flash freeze functionality is enabled.

The IrisSat CDH allows for over-the-air updates of the software.  
Below are the steps to prepare a software update file, see the IrisTerminal documentation for instructions on how to upload the file and initiate the update.
It is possible to generate an update file with only the eNVM content(i.e. the software), which is about 5 times smaller than the full image with the fpga data.
The golden image should be a full image, and the updates should be eNVM only (in most cases).

1. Build the "Production" version of the software in Softconsole. Click the arrow next to the hammer and select "Production".
2. In Libero, perform step 11 from above, making sure the latest Production binary is selected.
3. In Libero, generate the bitstream. Right click on the "Generate Bitstream" and configure here to check only eNVM or enVM and Fabric, depengin on what you want.
4. In the "Design Flow" panel on the left, select "Export Bistream".
5. In the "Export Bistream" options, give the file a name, and verify that the "SPI" format is enabled. Select "Fabric" and/or "eNVM" in the "File Types:" option. Make sure the "Export SPI Directory..." is enabled.
6. Click on "Specify SPI Directory" and make sure the golden and update addresses are set.
7. Also set the design version for each image. In general the golden image should be the same, and the update would be increased. It seems backlevel protection should be disabled by default, so the exact number shouldn't really matter.

