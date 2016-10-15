STM32-LMIC library
==================

This repository contains the IBM LMIC (LoraMAC-in-C) library, slightly modified to
match the frequency plan of TheThingsNetwork.

Supported hardware in this repository are the stm32 based IMST/WiMOD boards. Alternatives
are for instance the NUCLEO L152RE boards equipped with a SX1276MB1xAS LoRa radio board.
https://developer.mbed.org/components/SX1276MB1xAS.


Building
--------

For building and flashing, the GNU ARM cross compiler toolchain and OpenOCD are sufficient.

Enter the examples directory and edit projects.gmk for correct path to the GNU tools.

To build the modem for instance, enter examples/modem and type 'make'. The built firmware
modem.hex and modem.bin can be found in the build folder.

To flash the modem on an IMST board, enter 'script' folder and type:
> ./flash.sh ./stlink-adapter2.cfg ./../examples/modem/build/modem.hex 
where you have to adapt 'stlink-adapter2.cfg' to correctly identify
your stlink adapter.


DOCs:
-----

For more information, follow the documentation in the 'doc' folder.

