Overview
========
The i2c_polling_b2b_transfer_slave example shows how to use i2c driver as slave to do board to board transfer 
with a polling master:

In this example, one i2c instance as slave and another i2c instance on the other board as master. Master sends a 
piece of data to slave, and receive a piece of data from slave. This example checks if the data received from 
slave is correct.

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

Board settings
==============
I2C one board:
  + Transfer data from MASTER_BOARD to SLAVE_BOARD of I2C interface, I2C4 pins of MASTER_BOARD are connected with
    I2C4 pins of SLAVE_BOARD
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SLAVE_BOARD        CONNECTS TO          MASTER_BOARD
Pin Name   Board Location     Pin Name   Board Location
For I2C4 connection, refer to: https://variwiki.com/index.php?title=MCUXpresso&release=MCUXPRESSO_2.13.0_V1.0_DART-MX8M-MINI#Demos_pins 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
When the demo runs successfully, the following message is displayed in the terminal:

I2C board2board polling example -- Slave transfer.


Slave received data :
0x 0  0x 1  0x 2  0x 3  0x 4  0x 5  0x 6  0x 7
0x 8  0x 9  0x a  0x b  0x c  0x d  0x e  0x f
0x10  0x11  0x12  0x13  0x14  0x15  0x16  0x17
0x18  0x19  0x1a  0x1b  0x1c  0x1d  0x1e  0x1f


End of I2C example .
