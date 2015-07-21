# guardian\_node #

This Package contains the Guardian control functionality.

  * Author: Roman Navarro
  * License: BSD
  * Repository: gwam-ros-pkg

**The guardian\_node is the node in charge of the communication with the motors controller and the odometry update**

**Starting the guardian\_node**

`roslaunch guardian_node guardian_real.launch`


## Interaction with the Guardian Node ##

### The guardian\_node publishes the following topics (Default: 15Hz) ###

/guardian\_node/odom  - Publishes the odometry of the robot

/guardian\_node/state - Publishes the general state of the robot

### The guardian\_node subscribes to the following topis ###

/guardian\_node/command  - Read the velocity commands

/modbus\_io/input\_output - Read the current state of the inputs/outputs module