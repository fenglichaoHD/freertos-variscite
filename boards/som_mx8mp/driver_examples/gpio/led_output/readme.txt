Overview
========
The GPIO Example project is a demonstration program that uses the KSDK software to manipulate the general-purpose
outputs.
The example is supported by the set, clear, and toggle write-only registers for each port output data register. The 
example take turns to shine the LED.


Toolchain supported
===================
- IAR embedded Workbench  9.32.1
- GCC ARM Embedded  10.3.1

Hardware requirements
=====================
- Micro USB cable
- VAR-SOM-MX8M-PLUS SoM
- J-Link Debug Probe
- 12V power supply
- Personal Computer
- Oscilloscope

Board settings
==============
For GPIO output pin refer to: https://variwiki.com/index.php?title=MCUXpresso&release=MCUXPRESSO_2.13.0_V1.0_DART-MX8M-PLUS#Demos_pins.

#### Please note this application can't support running with Linux BSP! ####

Prepare the Demo
================
1.  Connect proper power supply and J-Link Debug Probe to the board, switch SW8 to power on the board
2.  Connect a proper cable between the host PC and the J18 header (pins UART4 TX, RX and GND) on the target board..
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
When the example runs successfully, the following message is displayed in the terminal:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 GPIO Driver example
 The LED is blinking.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

