# wam\_msgs #

This Package contains WAM specific message definitions.

  * Author: Kyle Maroney
  * License: BSD
  * Repository: gwam-ros-pkg
  * Source: svn https://gwam-ros-pkg.googlecode.com/svn/trunk/wam/wam_common/wam_msgs

# ROS Message Types #

**RTJointPos** - Type: float32`[]` joints, float32`[]` rate\_limits

**RTJointVel** - Type: float32`[]` velocities

**RTCartPos** - Type: float32`[3]` position

**RTCartVel** - Type: float32`[3]` direction, float32 magnitude

**RTOrtnPos** - Type: float32`[4]` quaternion

**RTOrtnVel** - Type: float32`[3]` angular, float32 magnitude