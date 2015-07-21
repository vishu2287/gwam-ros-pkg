# sphereptz #

  * Author: Román Navarro García
  * License: BSD

**The sphereptz manages the communication with the Logitech SphereCam**

This node moves the pan-tilt camera.

**Starting the sphereptz**

`roslaunch sphereptz test_sphereptz.launch`


## Interaction with the Sphereptz Node ##

### The sphereptz publishes the following topics (Default: 10Hz) ###

/sphereptz/ptz\_state  - Publishes the state of the pan & tilt


### The sphereptz subscribes to the following topis ###

/sphereptz/command\_ptz  - Read the pan-tilt commands (desired position)