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
- DART-MX8M-MINI SoM
- J-Link Debug Probe
- proper power supply
- Personal Computer
- Oscilloscope

Board settings
==============
For GPIO output pin refer to: https://variwiki.com/index.php?title=MCUXpresso&release=MCUXPRESSO_2.13.0_V1.0_DART-MX8M-MINI#Demos_pins

#### Please note this application can't support running with Linux BSP! ####

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
5.  Launch the debugger in your IDE to begin running the demo.

Running the demo
================
When the example runs successfully, the following message is displayed in the terminal:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 GPIO Driver example
 The LED is blinking.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

