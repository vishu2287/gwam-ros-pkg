# wam\_srvs #

This Package contains WAM / BarrettHand specific service definitions.

  * Author: Kyle Maroney
  * License: BSD
  * Repository: gwam-ros-pkg
  * Source: svn https://gwam-ros-pkg.googlecode.com/svn/trunk/wam/wam_common/wam_srvs

# ROS Service Types #

**GravityComp** - Request: bool gravity

**Hold** - Request: bool hold

**JointMove** - Request: float32`[]` joints (Must be equal to DOF of WAM)

**PoseMove** - Request: geometry\_msgs/Pose pose

**CartPosMove** - Request: float32`[3]` position

**OrtnMove** - Request: float32`[4]` orientation

**BHandFingerPos** - Request: int8 finger, float32 radians

**BHandGraspPos** - Request: float32 radians

**BHandSpreadPos** - Request: float32 radians

**BHandFingerVel** - Request: int8 finger, float32 velocity

**BHandGraspVel** - Request: float32 velocity

**BHandSpreadVel** - Request: float32 velocity