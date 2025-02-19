Overview
========
The freertos_ecspi_loopback demo shows how the ecspi do a loopback transfer internally in FreeRTOS.
The ECSPI connects the transmitter and receiver sections internally, and the data shifted out from the 
most-significant bit of the shift register is looped back into the least-significant bit of the Shift register. 
In this way, a self-test of the complete transmit/receive path can be made. The output pins are not affected, 
and the input pins are ignored.

Toolchain supported
===================
- IAR embedded Workbench  9.32.1
- GCC ARM Embedded  10.3.1

Hardware requirements
=====================
- Micro USB cable
- MCIMX8MM6-EVK  board
- J-Link Debug Probe
- proper power supply
- Personal Computer

Board settings
==============
No special settings are required.


Prepare the Demo
================
1.  Connect proper power supply and J-Link Debug Probe to the board, switch SW8(DT8CustomBoard)/SW7(SymphonyBoard) to power on the board
2.  Connect a proper cable between the host PC and the J12 header(DT8CustomBoard)/J18 header(SymphonyBoard), (pins UART3 TX, RX and GND) on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
If the demo run successfully, the below log will be print in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
***FreeRTOS ECSPI Loopback Demo***

This demo is a loopback transfer test for ECSPI.
The ECSPI will connect the transmitter and receiver sections internally.
So, there is no need to connect the MOSI and MISO pins.

FreeRTOS ECSPI loopback test pass!
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
