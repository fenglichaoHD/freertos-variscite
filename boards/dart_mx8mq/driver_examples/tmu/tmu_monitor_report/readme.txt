Overview
========
The TMU example shows how to configure TMU register to monitor and report the temperature from one or
more remote temperature measurement sites located on the chip.

TMU has access to multiple temperature measurement sites strategically located on the
chip. It monitors these sites and can signal an alarm if a programmed threshold is ever
exceeded.

Note: Before monitoring the temperature, we must programming the calibration table.

Toolchain supported
===================
- IAR embedded Workbench  9.32.1
- GCC ARM Embedded  10.3.1

Hardware requirements
=====================
- Micro USB cable
- DART-MX8M SoM
- J-Link Debug Probe
- proper power supply
- Personal Computer

Board settings
==============
No special settings are required.



Prepare the Demo
================
1.  Connect proper power supply and J-Link Debug Probe to the board, switch SW8 to power on the board
2.  Connect a proper cable between the host PC and the J12 header (pins UART2 TX, RX and GND) on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Launch the debugger in your IDE to begin running the demo.

Running the demo
================
The log below shows the output of the hello world demo in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TMU monitor report example.
site 1 immediate temperature is too high. 41°C
site 1 immediate temperature is too high. 41°C
site 0 immediate temperature is too high. 41°C
site 1 immediate temperature is too high. 42°C
site 0 immediate temperature is too high. 41°C
site 1 immediate temperature is too high. 42°C
...
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
