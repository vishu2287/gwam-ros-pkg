# modbus\_io #

This Package contains the mod-bus I/O module control functionality.

  * Author: Román Navarro García
  * License: BSD


**The modbus\_io node manages the communication with with an I/O module via modbus**

The module offers a range of 8 digital I/O and 2 analog I/O

By default, the digital input 1 is reserved


**Starting the modbus\_io node**

`roslaunch modbus_io test_io.launch`


## Interaction with the Modbus\_io Node ##

### The modbus\_io publishes the following topics (Default: 20Hz) ###

/input\_output  - Publishes the current state of all the I/O


### The modbus\_io offers the following services ###

/write\_digital\_output - _Type write\_digital\_output.srv_

/write\_analog\_output  - _Type write\_analog\_output.srv_