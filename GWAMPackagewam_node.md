# wam\_node #

This Package contains Barrett WAM / BarrettHand control functionality.

  * Author: Kyle Maroney
  * License: BSD
  * Repository: gwam-ros-pkg
  * Source: svn https://gwam-ros-pkg.googlecode.com/svn/trunk/wam/wam_robot/wam_node

**The wam\_node is the primary communication node between the WAM / BarrettHand and ROS.**

**This node is designed to be launched on the WAM only!**

The wam\_node was designed to abstract the Libbarrett functionality exposed by the [libbarrett\_ros package](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagelibbarrett_ros) into a simple, fully functional ROS node.  Allowing for both real-time and non real-time control of the Barrett WAM.

The wam\_node package is self aware of robot configuration.  This means that if you have a 7-dof WAM configuration but want to use the robot without the BarrettHand the wam\_node will take care of this.  Not publishing or subscribing to the services and messages available for a WAM with a BarrettHand.

**Starting the wam\_node**
```
roslaunch wam_bringup wam_guardian.launch
```

## Interaction with the WAM / BarrettHand ##

### The WAM Node Publishes the following topics (default: 100Hz) ###

wam/joint\_states - Publishes the current angle in radians for each joint present on the wam.

wam/pose - Publishes the current pose (cartesian position and orientation).

bhand/joint\_states - Publishes the current angle in radians for each joint of the BarrettHand.

### The WAM Node Subscribes to the following topics - primarily for RT Control ###

wam/jnt\_pos\_cmd _- type wam\_msgs/RTJointPos_

wam/jnt\_trq\_cmd _- type wam\_msgs/RTJointTrq_

wam/jnt\_vel\_cmd _- type wam\_msgs/RTJointVel_

wam/cart\_pos\_cmd _- type wam\_msgs/RTCartPos_

wam/cart\_vel\_cmd _- type wam\_msgs/RTCartVel_

(TBD)wam/ortn\_pos\_cmd _- type wam\_msgs/RTOrtnPos_

wam/ortn\_vel\_cmd _- type wam\_msgs/RTOrtnVel_

wam/pose\_vel\_cmd _- type wam\_msgs/RTPoseVel_


### The WAM Node offers the following Service Calls ###

**WAM**

wam/gravity\_comp _- type wam\_srvs/GravityComp_

wam/go\_home _- type std\_srvs/Empty_

wam/hold\_joint\_pos _- type wam\_srvs/Hold_

wam/hold\_cart\_pos _- type wam\_srvs/Hold_

wam/hold\_ortn _- type wam\_srvs/Hold_

wam/joint\_move _- type wam\_srvs/JointMove_

(TBD) wam/pose\_move _- type wam\_srvs/PoseMove_

wam/cart\_move _- type wam\_srvs/CartPosMove_

wam/ortn\_move _- type wam\_srvs/OrtnMove_

**BarrettHand**

bhand/open\_grasp _- type std\_srvs/Empty_

bhand/close\_grasp _- type std\_srvs/Empty_

bhand/open\_spread _- type std\_srvs/Empty_

bhand/close\_spread _- type std\_srvs/Empty_

bhand/finger\_pos _- type wam\_srvs/BHandFingerPos_

bhand/grasp\_pos _- type wam\_srvs/BHandGraspPos_

bhand/spread\_pos _- type wam\_srvs/BHandSpreadPos_

bhand/finger\_vel _- type wam\_srvs/BHandFingerVel_

bhand/grasp\_vel _- type wam\_srvs/BHandGraspVel_

bhand/spread\_vel _- type wam\_srvs/BHandSpreadVel_