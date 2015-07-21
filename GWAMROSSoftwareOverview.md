# gwam-ros-pkg Software Overview #


---

**The gwam-ros-pkg is a ROS repository for control of the Guardian-WAM robot in both simulation and the real world.**

---


For Installation of gwam-ros-pkg please refer to the [gwam-ros-pkg Installation Instructions](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMROSInstallation)

## Capability Overview ##

### Basic Configuration ###
| **Component**      | **ROS package/stack** |
|:-------------------|:----------------------|
|WAM-Specific Messages | [wam\_msgs](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagewam_msgs) |
|WAM-Specific Services | [wam\_srvs](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagewam_srvs) |
|WAM Libbarrett Wrapper | [libbarrett\_ros](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagelibbarrett_ros) |
| WAM Model (URDF)   | [wam\_description](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagewam_description) |
| Guardian Model (URDF) | [guardian\_description](guardian_description.md) |
| GWAM Model (URDF)  | [gwam\_description](gwam_description.md) |

### Hardware Drivers and Simulation ###
| **Component**      | **ROS package/stack** |
|:-------------------|:----------------------|
| WAM / BHand Node   | [wam\_node](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagewam_node) |
| Guardian / Guardian Node | [guardian\_node](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackageguardian_node) |
| Guardian / Sphere Cam | [usb\_cam](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagesphere_cam) |
| Guardian / Sphere Pan-tilt | [sphere\_ptz](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagesphere_ptz) |
| Guardian / Module I/O |  [modbus\_io](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagemodbus_io) |
| Guardian / IMU     |  [microstrain\_imu](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagemicrostrain_imu) |
| GWAM Simulation    | [gwam\_bringup](gwam_bringup.md) |

### High-Level Capabilities ###
| **Component**      | **ROS package/stack** |
|:-------------------|:----------------------|
| Teleop             | [gwam\_joystick\_teleop](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMJoystickTeleop) |
| Navigation         | [guardian\_navigation](guardian_navigation.md) |
| Arm Navigation     | [TBD](TBD.md)         |



Now that you understand the layout of the software, please continue to the [GWAM Tutorials](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsMainPage)