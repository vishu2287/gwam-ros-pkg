# Guardian-WAM Tutorial: Controlling the WAM with ROS Service Calls #

**The following tutorial will further understanding of the ROS Services exposed by the wam\_node and allow us to control the WAM with Joint and Cartesian Position Commands.**

**Please bring up the entire Guardian-WAM System before continuing with this tutorial**

> - Turn on the Guardian-WAM Master Power, WAM power, start the Guardian PC, and reset the power to the motor controller.

> - Once the Guardian system is online, please SSH into the WAM PC and launch the wam\_node

```
ssh robot@WAM
#password: WAM

roslaunch wam_bringup wam_guardian.launch
```

Your system should now be initialized and running, please continue with the tutorial.

## Instructions ##

In a terminal:
```
#First verify that your ROS_MASTER_URI is set to the guardian-wam
env | grep ROS_MASTER_URI

#if your ROS_MASTER_URI does not point to http://guardian-wam:11311
export ROS_MASTER_URI = http://guardian-wam:11311
#or edit your ROS Environment file (usually ~/ros/gwam_env.ros) to point to the guardian-wam pc
```

View all of the ROS Services available that correspond to controlling the WAM.
```
#The following will show all of the rosservices available
rosservice list

#This tutorial only pertains to rosservices concerning the WAM
rosservice list | grep wam

```

We will now step through each of the WAM Service Commands to understand what they do.

In a terminal please do these commands one at a time:
```
rosservice call /wam/go_home
```
This command will return the WAM to its home position at any time.

The go\_home command will open the grasp, close the spread, move the robot to its zero joint positions, then gently return the robot to its location when the wam\_node was started.

```
rosservice call /wam/hold_joint_pos false
```
By default the previous go\_home command holds it's joint angles in the home position.

When issuing the hold\_joint\_pos command we can give it an argument true or false.  This will turn on and off holding the current joint position.

You should now be able to move the WAM around in free space, note that gravity compensation is on by default.

At any point with the WAM you can make the wam hold its current joint position with the command.
```
rosservice call /wam/hold_joint_pos true
## We can also shut off the joint hold using
rosservice call /wam/hold_joint_pos false
```

Similarly, we can hold our cartesian position or end-effector orientation in space using:
```
# To hold cartesian position
rosservice call /wam/hold_cart_pos true
# or 
rosservice call /wam/hold_cart_pos false

# To hold end-effector orientation
rosservice call /wam/hold_ortn true
# or 
rosservice call /wam/hold_ortn false
```

We can command the WAM to move to a joint position request **in Radians**
```
#For a 7-DOF WAM
rosservice call /wam/joint_move [0,0,0,0,0,0,0]
rosservice call /wam/joint_move [0.5,0.5,0.5,0.5,0.5,0.5,0.5]
rosservice call /wam/joint_move [1.0,0.5,0.5,0.5,0.5,0.5,0.5]
rosservice call /wam/joint_move [-1.0,0.5,0.5,0.5,0.5,0.5,0.5]

#For a 4-DOF WAM
rosservice call /wam/joint_move [0,0,0,0]
rosservice call /wam/joint_move [0.5,0.5,0.5,0.5]
rosservice call /wam/joint_move [1.0,0.5,0.5,0.5]
rosservice call /wam/joint_move [-1.0,0.5,0.5,0.5]
```
The joint\_move command moves the WAM Joints to the angles requested **in radians**

We can also do cartesian position moves **in Meters**
```
rosservice call /wam/cart_move [0.5,0.5,0.5]
```
The cart\_move command moves the WAM end-effector to the cartesian position requested.  Please interact with the WAM in this pose and notice the ability move individual joints by hand while the WAM maintains its end effector cartesian position.

We can also do end effector orientation moves **in Quaternion**
```
rosservice call /wam/ortn_move [0.0 0.0 0.0 1.0]
```
The ortn\_move command move the WAM end-effector to the orientation command requested.  Please interact with the WAM under the orientation command constraint.  Notice that you can freely move the WAMs position and joints in space while the end-effector tries to maintain its orientation.

Please send the WAM back to its home position before continuing:
```
rosservice call /wam/go_home
```

Now that you understand all of the WAM services available, and how to command them.  Please continue to the next tutorial for controlling the BarrettHand in a similar manner:  **[Controlling the BarrettHand with Service Calls](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsBHANDCntrl)**