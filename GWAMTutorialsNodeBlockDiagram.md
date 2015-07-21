# Guardian-WAM Tutorial: Guardian-WAM Hardware / Software Schematics #

**The following tutorial describes the hardware and software schematics for the Guardian-WAM System.**

## Guardian-WAM Schematics ##

### Guardian-WAM System Schematic ###

![http://i.imgur.com/q0lGf.png](http://i.imgur.com/q0lGf.png)

The above image is a block diagram schematic of all the components that make up the Guardian-WAM system.  Data Communication between components is portrayed with blue arrows, while power to the system is portrayed with red arrows.  The dashed arrows represent the ability for multiple computers to access the Guardian-WAM hardware and software system simultaneously.

### Guardian-WAM Software Schematic ###

![http://i.imgur.com/stMGi.png](http://i.imgur.com/stMGi.png)

The above image describes the ROS nodes that make up the default Guardian-WAM ROS software launched.  Upon starting the Guardian system a ROS Master is started on the Guardian PC.  A script is launched on startup that initializes all of the above nodes described in blue.  These nodes all exist and are controlled on the Guardian PC.  The yellow wam\_node denotes the node that needs to be launched on the WAM robot.  This node will initialize the WAM robot exposing all of the rostopics and services produced by the WAM.

For instructions on launching the wam\_node on the WAM robot please see the previous tutorial:

[Guardian-WAM Quick Start Guide Tutorial](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsGWAMQuickStart)

Now that you have a more in depth understanding of the Guardian-WAM Hardware / Software configuration, please continue to the next tutorial:

[Understanding the Guardian-WAM Nodes/Topics/Services](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsUnderstandingGWAMNodes)