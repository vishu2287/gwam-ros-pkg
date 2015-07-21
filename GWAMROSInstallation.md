# Installation Instructions for the gwam-ros-pkg Repository #

To Install the gwam-ros-pkg Software for the Guardian-WAM System.  Navigate to a working ROS folder, (Typically ~/ros).  Verify that this folder is in your ROS\_PACKAGE\_PATH.
```
env | grep ROS_PACKAGE_PATH
```
If it is, please continue on. If your current directory is not in your ROS\_PACKAGE\_PATH please follow the instructions [here](http://code.google.com/p/gwam-ros-pkg/wiki/InitialSetupInstructionsOffboardPC) for setting up your environment correctly.  Also, for further information on your ROS\_PACKAGE\_PATH visit: http://ros.org/wiki/ROS/EnvironmentVariables.

**Install gwam-ros-pkg**

```
svn checkout http://gwam-ros-pkg.googlecode.com/svn/trunk/ gwam-ros-pkg
```

Now that you have successfully checked out the gwam-ros-pkg please continue to our [gwam-ros-pkg Software Overview](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMROSSoftwareOverview)