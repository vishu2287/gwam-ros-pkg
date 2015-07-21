# libbarrett\_ros #

This Package Contains a Third-Party Wrapper for Libbarrett - A C++ Robot Control Library Developed by Barrett Technology.

  * Author: Kyle Maroney, Barrett Technology Inc.
  * License: BSD
  * Repository: gwam-ros-pkg
  * Source: svn https://gwam-ros-pkg.googlecode.com/svn/trunk/wam/wam_robot/libbarrett_ros

**The libbarrett\_ros package was designed to wrap Libbarrett. allowing for linking against its libraries for both real-time and non real-time control of Barrett Technology's products, the WAM, BarrettHand, and Force Torque Sensor**

The libbarrett\_ros package should only be compiled on either the WAM onboard PC-104, or a WAM external real-time control PC.

Compilation Instructions:
```
roscd libbarrett_ros
rosmake
```