LoraWAN EasyNode
================

<img width="100%" align="left" src="hardware\easynode-complete.jpg">

What does it do?
================
The LoRaWAN EasyNode is extremely low-cost and easy to build (all hand-solderable) battery-powered (3xAAA cells) device that has these capabilities:

* Temperature measurement from the on-board sensor
* Device configuration via SD card text file
* Maintaining Time (with its built-in Real Time Clock) with a coin cell battery
* Saving data in CSV format to SD card with timestamps (new file every 24 hours)
* Sending temperature to a LoRaWAN network (i.e. data uplink)
* Receiving ON/OFF for two open collector outputs from the LoRaWAN network (i.e. data downlink)
* Device sleep and wake up at time intervals

<img width="100%" align="left" src="hardware\lowawan-solution-overview.png">

Building It
===========
Upload the board zip file (it contains Gerber files) to any PCB manufacturer (such as JLC PCB or PCBWay) website, and it should cost under $10 for five boards.

All the parts are listed in the [Bill of Materials](https://github.com/shabaz123/easynode/tree/main/hardware). At the time of writing all parts are currently available despite the world semiconductor shortage, and many of them can be substituted with others if they become unavailable. The SD card socket, coin cell holder, and the radio module (RFM95W) are available from AliExpress.

Solder the parts in the order they are listed in the bill of materials. The security chip U6 is small and harder to solder than the rest of the parts, but it is optional (it is not currently supported in the software) so it can be omitted for now if you're just evaluating other LoRaWAN aspects.

<img width="100%" align="left" src="hardware\easynode-front-render-rev1-3.png">

<img width="100%" align="left" src="hardware\easynode-underside-render-rev1-3.png">

MSP430 Firmware
===============
There are two chips on the EasyNode board that need to be programmed. The first is the MSP430 chip which acts as a power controller to power-on and power-off the rest of the board to conserve power between sensor measurements.

To program that chip, some sort of programmer is required. These options work:

* MSP-EXP430G2ET Launchpad Board
* MSP-FET430UIF Debugger
* MSP-FET Debugger

The first option is cheapest. The MSP-EXP430G2ET board has a socket that can be used to program the chip, and then the chip can be removed and plugged onto the EasyNode board. With the other two options, the chip can be programmed in-situ on the EasyNode board, by merely connecting the Debugger to the EasyNode 4-pin connector labelled J1.

<img width="100%" align="left" src="hardware\msp-fet-connections-annotated.jpg">

With any of the three options, the programming process is identical; download and install Code Composer Studio (CCS) on your PC, open the [System Power Controller project](https://github.com/shabaz123/system_power_controller/tree/main/sys_pwr_controller) and then connect the Launchpad or Debugger using a USB cable to the PC, and then press the hammer icon in CCS to build the code, and then press the bug icon to transfer the firmware.

<img width="100%" align="left" src="hardware\ccs-how-to-use.png">

Note that if you're using a Debugger connected to J1, then the board must first be powered up, using the USB connector on the Pi Pico, regardless of if the board also has a coin cell and AAA batteries connected.

Pi Pico Firmware
================
The Pi Pico can be programmed without any special equipment. Connect the USB connection to a PC. Hold down the only button on the Pico module (it is a white button labeled BOOTSEL), as well as the RST button on the EasyNode board, and then release the RST button on the Pico board, wait a few seconds and then release the BOOTSEL button. This will put the Pico into Bootloader mode and you’ll see a drive letter appear on the PC. Now you can drag and drop the firmware (it is the .uf2 suffixed file available on GitHub in the `prebuilt` folder) onto the Pico.

If you wish to build the firmware yourself, then follow these steps:

* Create a development folder on your PC, for instance `C:\development\pico`
* Install the Pico SDK, such that the path for it is `C:\development\pico\pico-sdk`
* From the `C:\development\pico` folder, type the following:

```
git clone https://github.com/shabaz123/easynode.git
cd easynode
git clone --recurse-submodules https://github.com/sandeepmistry/pico-lorawan.git
git clone --recurse-submodules https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico.git
```

* Open the project in CLion (it is free for 30 days; if you don’t wish to use CLion, you can build directly from the command line, or using Visual Code, but it’s pretty easy to use CLion), and in the **Open Project Wizard** window **Environment** box, type:

```
PICO_SDK_PATH=c:\development\pico\pico-sdk
```

<img width="100%" align="left" src="hardware\clion-env-path.png">

* Press OK and wait a minute or so for CLion to build up its index.
* Click on the hammer icon, and the code should be built.
* Once the code builds successfully, you'll need to make one change in the LoRaWAN stack. Go to the following file:

    `easynode\pico-lorawan\src\boards\rp2040\sx1276-board.c`
    
and in this function:

    `static uint8_t SX1276GetPaSelect( int8_t power )`
    
(it will be around line 205), add this as the first line in the function:

```
return RF_PACONFIG_PASELECT_PABOOST; // fix for RFM95W
```

* Click the hammer icon to rebuild the project. The .uf2 file will be in the `easynode\cmake-build-debug` folder, ready for transferring to the Pi Pico as described above in the Pico Firmware section.

Configuring It
==============

Place the following `config.txt` file on a FAT formatted SD card, and edit to suit requirements.

```
# example config.txt file

# choose your region
# lorawan_region aus
lorawan_region europe
# lorawan_region india
# lorawan_region north_amer

# 16-char device EUI
lorawan_device_eui 1234560000000001

# 16-char app or join EUI
lorawan_app_or_join_eui 1234990000000100

# 32-char app key
lorawan_app_key 112233445566778899AABBCCDDEEFF00

# wake interval
# realistically needs to be longer than 45 sec
# if longer than 255 sec, then only values wholly
# divisible by 60 are usable, e.g.
# 300, 360, 420 are fine for 5, 6, 7 minutes for example
wake_after_sec 300

# seed (can be random) used as a secret 8 hex bytes base64
# can use https://base64.guru/converter/encode/hex
random_seed ESIzRFVmd4g=

# put any time configuration here
date 25
month 7
year 2022
time 11:00:00
```

The config.txt file is only read by the EasyNode if the INIT button is pressed while the Pi Pico is booting up. So, to do that, plug in the USB connection on the Pi Pico (you can also optionally view debug by using PuTTY or any other terminal software, at 115200 baud, on the USB serial connection), then hold down the INIT button, and press the RST button and release the RST button, and then after 10 seconds or so, release the INIT button. The serial debug will show that the config.txt file has been read. The file contents are transferred to the Pico's Flash memory. After that, the SD card is only used for data logging, and the config.txt file can be deleted if desired for security reasons.

Extending the EasyNode
======================
There are a couple of open drain outputs on J2, for controlling devices such as relays. The outputs are implemented in code and ready to use; just send a byte from the LoRaWAN network, and the two least significant bits of it will be used to control the two open drain outputs.

Additional I2C sensors can be attached to the header pins labeled J2. There are a couple of inputs too, but I have not implemented any code to support that yet.

Appendix: Hardware Change Log
=============================

**Rev 1.0 - shabaz - July 2022** Initial version, requires a couple of bodge wires

**Rev 1.2 - shabaz - July 2022** Fixed the wire bodges, and added a Security Controller (U6) and added a 2-pin connector for MIC280 external temperature sensor.

**Rev 1.3 - shabaz - July 2022** Removed MIC280 and its 2-pin connector, and replaced with STS21 which is a better temperature sensor.




