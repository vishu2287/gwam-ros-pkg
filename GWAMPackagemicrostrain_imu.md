# microstrain\_3dmgx2\_imu #

This Package contains the control functionality of IMU Microstrain 3DMGX2.

  * Author: Román Navarro García
  * License: BSD


**The microstrain\_3dmgx2\_imu manages the communication with the imu**

This node gets the information from the device and publishes it.

**Starting the microstrain\_3dmgx2\_imu**

`roslaunch microstrain_3dmgx2_imu microstrain_3dmgx2.launch`


## Interaction with the IMU Node ##

### The microstrain\_3dmgx2\_imu publishes the following topics (Default: 100Hz) ###

/diagnostics

/imu/data -- Type sensor\_msgs::Imu

/imu/is\_calibrated

### The microstrain\_3dmgx2\_imu offers the following services ###

/imu/calibrate

/microstrain\_3dmgx2\_node/add\_offset