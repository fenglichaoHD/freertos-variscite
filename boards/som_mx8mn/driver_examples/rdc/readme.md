Overview
========

The RDC example shows how to control the peripheral and memory region
asscess policy using RDC and RDC_SEMA42

SDK version
===========
- Version: 2.15.000

Toolchain supported
===================
- IAR embedded Workbench  9.40.1
- GCC ARM Embedded  12.2

Hardware requirements
=====================
- Micro USB cable
- VAR-SOM-MX8M-NANO SoM
- J-Link Debug Probe
- proper power supply
- Personal Computer

Board settings
==============
No special settings are required.

#### Please note this application can't support running with Uboot! and accordingly it does not support Flash target and DDR target! ####
This example aims to show the basic usage of the IP's function, some of the used Resources are assigned to Cortex-A core by uboot.

Prepare the Demo
================
1.  Connect proper power supply and J-Link Debug Probe to the board, switch SW7 to power on the board
2.  Connect a proper cable between the host PC and the J18 header, (pins UART3 TX, RX and GND) on the target board.
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
The log below is shown in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RDC Example:
RDC Peripheral access control
RDC Peripheral access control with SEMA42
RDC memory region access control

RDC Example Success
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
