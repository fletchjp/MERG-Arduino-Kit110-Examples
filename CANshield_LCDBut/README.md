<img align="right" src="arduino_cbus_logo.png"  width="150" height="75">

# CANshield_LCDBut

This is an example of an application of the Arduino CAN Shield (MERG Kit Locker #110).

The example code runs on an Arduino UNO with a CAN shield and a DFRobot shield with a 2 by 16 LCD display and buttons.

The example program contains code to send CBUS events when the buttons on the shield are pressed. Details are given below (see Outgoing events).

This examples is fairly complex code as it has to support both the reading of the input buttons and the send information to the LCD display. 

https://www.dfrobot.com/product-51.html (DFR0009)

There are some libraries which are used to control the display. They are all available in the Arduino IDE.

 - IoAbstraction   https://www.thecoderscorner.com/products/arduino-libraries/io-abstraction/
 - TaskManagerIO   https://www.thecoderscorner.com/products/arduino-libraries/taskmanager-io/
 - LiquidCrystalIO https://www.thecoderscorner.com/products/arduino-libraries/liquidcrystal-io/

These are all from the same source and work together well. They provide a way of handling the display, buttons and task management.

## Please note

The CANshield button and LEDs not available for CBUS configuration in this code.

This is because the pins for the LEDs and buttons are also used by the display, quite apart from them being hidden behind the display.

All use of the button and LEDs has been removed from the code of CANshield as otherwise the display does not function correctly.

## Outgoing events

The example can send CBUS events from the buttons as follows. The numbers are the CBUS event numbers. These can be taught using FCU to any module which is to use the information.

 - 1 RIGHT
 - 2 LEFT
 - 3 UP
 - 4 DOWN
 - 5 SELECT
 
  The RST button resets the Arduino which restarts the connection to CBUS and refreshes the display.
  
## Incoming events

The example can now receive events as well. These events display a message on the top line of the display.

The idea is to simulate a situation where an error message of some sort is being sent from somewhere else on the CBUS system. The error messages are stored in this code and the error message intructs this code what message to display. This is done either with a specific event for each message or else with a message with a number which is an index for the message to display.

Both ON and OFF messages are supported. The OFF message turns off the error message.

NOTE: The CBUS specification supports messages with extra data. They are NOT supported by FCU and the messages have to come from another Arduino code which does support them.

Event numbers supported as both long and short events. The events need to be taught to the module in the usual way for CBUS.

 - 100 ON sends 1 byte of data to choose the message
 - 101 ON turns on display 1
 
## Query and response

The CBUS specification supports messages between modules which allow for the sending of a query message to which a module can reply. They are NOT supported by FCU and the messages have to come from another Arduino code which does support them.

Also they cannot be taught as events and handling has to be included in the code. This is done using a framehandler code which can be set up to process only particular events and codes.

The example query supports the situation where a module on the system needs information on what error is being displayed.

 - 100 A query to event number 100 can be sent from another module. This module replies with a response which includes the number of the error message being displayed.
 
## More information

An Arduino program using the Arduino CAN Shield (MERG Kit Locker #110).
Information about the kit can be found at https://www.merg.org.uk/merg_wiki/doku.php?id=kits:110

The for this application it has to be built as a shield to fit onto an Arduino UNO.

This sketch provides the code which will enable users of the Arduino CAN Shield to test functionality of the shield.
Sketch is the Arduino name for the program code which is run on an Arduino.

Key Features:
- MERG CBUS interface.
- This can be configured to a CBUS network using the FLiM Configuration Utility (FCU) which only runs on Windows computers.
- This can be done using the Serial Monitor as described below.
- The puish button method is not available for this code.

## Overview

The program is written in C++ but you do not need to understand this to use the program.

## Loading the code and libraries

In order to load the code it is necessary to load the Arduino IDE (Interactive Development Environment) onto a computer.

There are currently two IDE versions, the older 1.8.19 and the newer 2.2.1. Either will do for this.
The IDE is available for different versions of Windows and also for other systems.

The IDE will manage both the code and also several libraries which can be loaded using a Library Manager.

All of the libraries needed for CANshield are available through the Library Manager which also prompts with updates for installed libraries.
The user installs libraries into a fixed location which has all of the libraries which have been loaded, available for any sketch.

Whenever a sketch is to be used the Arduino IDE calls a C++ compiler which compiles the code and a copy of each of the libraries needed. This can be done as a check that it allo works and then loaded onto the Arduino using the connection.

On a Windows computer the USB connection to the computer will be via a COM port and it is necessary to tell the Arduino IDE which COM port to use.

## Using CANshield

The MCP2515 interface requires five Arduino pins to be allocated. Three of these are fixed
in the architecture of the Arduino processor. One pin must be connected to an interrupt
capable Arduino pin. On the shield, this is set to pin 2 but a jumper can be removed and a
wire link used to connect to another interrupt pin if desired.  This may be appropriate
when using a Mega with additional interrupt inputs.

The Chip Select pin on the shield is connected to pin 10.  Once again, by removing the relevant
jumper, another pin can be wire linked for this function.

If you change the interrupt or chip select pins, make sure that you update the relevant pin 
allocations in the sketch.

If the MERG Kit 110 CAN Shield is used, the following pins are connected by default:

Pin | Description
--- | ---
Digital pin 2 | Interupt CAN
Digital pin 10| (SS)    CS    CAN
Digital pin 11| (MOSI)  SI    CAN
Digital pin 12| (MISO)  SO    CAN
Digital pin 13| (SCK)   Sck   CAN

Using the CAN Shield, the following pins are used for CBUS Initialisation:

NOTE: For this application the pin 10 is taken by the display shield and CS has to be made 15 instead.

**It is the users responsibility that the total current that the Arduino is asked to supply 
stays within the capacity of the on board regulator.  Failure to do this will result in 
terminal damage to your Arduino.**

Pins defined as inputs are active low.  That is to say that they are pulled up by an 
internal resistor. The input switch should connect the pin to 0 Volts.

Pins defined as outputs are active high.  They will source current to (say) an LED. It is 
important that a suitable current limiting resistor is fitted between the pin and the LED 
anode.  The LED cathode should be connected to ground.

### Library Dependencies

The following third party libraries are required for the basic CANshield code:

Library | Purpose
---------------|-----------------
Streaming  |*C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)*
Bounce2    |*Debounce of switch inputs*
ACAN2515   |*library to support the MCP2515/25625 CAN controller IC*
CBUS2515   |*CAN controller and CBUS class*
CBUSconfig |*module configuration*
CBUS       |*CBUS Class*
CBUSSwitch   |*Manage CBUS switch*
CBUSLED      |*Manage CBUS indicator yellow/green LEDs*

## The Serial Monitor

If your Arduino is connected to the Arduino IDE on your computer via the USB port, you can
access information by sending a character from the IDE.  The function of these characters
is as follows:

### 'r'
This character will cause the module to renegotiate its CBUS status by requesting a node number.
The FCU will respond as it would for any other unrecognised module.

### 'z'
This character needs to be sent twice within 2 seconds so that its action is confirmed.
This will reset the module and clear the EEPROM.  It should thus be used with care.

Other information is available using the serial monitor using other commands:

### 'n'
This character will return the node configuration.

### 'e'
This character will return the learned event table in the EEPROM.

### 'v'
This character will return the node variables.

### 'c'
This character will return the CAN bus status.

### 'h'
This character will return the event hash table.

### 'y'
This character will reset the CAN bus and CBUS message processing.

### '\*'
This character will reboot the module.

### 'm'
This character will return the amount of free memory. 
 
 
 
 
 
