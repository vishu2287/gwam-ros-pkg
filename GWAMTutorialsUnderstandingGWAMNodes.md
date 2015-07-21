# Guardian-WAM Tutorial: Understanding the Guardian-WAM Nodes/Services/Topics #

**The following tutorial will help in better understanding the Guardian-WAM Nodes, Services, and Topics on the ROS Graph.**

**Please bring up the entire Guardian-WAM System before continuing with this tutorial**

> - Turn on the Guardian-WAM Master Power, WAM power, start the Guardian PC, and reset the power to the motor controller.

> - Once the Guardian system is online, please SSH into the WAM PC and launch the wam\_node

```
ssh robot@WAM
#password: WAM

roslaunch wam_bringup wam_guardian.launch
```

Your system should now be initialized and running, please continue with the tutorial.

## Guardian-WAM Nodes ##

ROS Nodes are the processes that perform computation.  As seen in the previous [tutorial](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsNodeBlockDiagram) a number of nodes are launched on the Guardian-WAM Startup.  We also start the wam\_node on the ROS graph by logging into the WAM and launching it.

To view all of the active guardian-wam nodes:
```
#First verify that your ROS_MASTER_URI is set to the guardian-wam
env | grep ROS_MASTER_URI

#if your ROS_MASTER_URI does not point to http://guardian-wam:11311
export ROS_MASTER_URI = http://guardian-wam:11311
#or edit your ROS Environment file (usually ~/ros/gwam_env.ros) to point to the guardian-wam pc

#The following displays all of the ROS nodes that are currently running on the Guardian-WAM

rosnode list
```

Most of these nodes should look familiar as they describe components of the Guardian-WAM system.

Future ROS nodes that a user may write will also be displayed with the above command.

The most important nodes of the guardian-ros-pkg are the wam\_node and guardian\_node.  The wam\_node abstracts all the functionality of the WAM and BarrettHand, while the guardian\_node exposes control of the Guardian mobile base.

These nodes are described in great detail in the software overviews:

[wam\_node](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackagewam_node) & [guardian\_node](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMPackageguardian_node)

Another interesting tool for visualization of our ROS Graph is rxgraph.

To view the current ROS Computation Graph in a terminal type:
```
rxgraph
```

This visualization will show the inter-connectivity of the various processes on the ROS Graph.  By clicking on individual nodes you can view the topics and services that node is publishing and subscribing to.

For more information of ROS Nodes and rxgraph please visit the following:
[ROS Nodes](http://www.ros.org/wiki/Nodes) , [rxgraph](http://www.ros.org/wiki/rxgraph) & [ROS Nodes Tutorial](http://www.ros.org/wiki/ROS/Tutorials/UnderstandingNodes)

## Guardian-WAM Topics ##

ROS Topics are buses over which data structures known as [messages](http://www.ros.org/wiki/Messages) are passed.  ROS Topics use publish/subscriber methods and separate the publishing of messages from their consumption.  Topics published and subscribed to on the Guardian-WAM system follow typical ROS symantics for areas such as commanding linear and angular velocity to the Guardian, and publishing of joint angles, effort and velocities of the WAM.

To view all of the rostopics on the ROS Graph in a terminal:
```
rostopic list
```

You will see a large number of topics available for control of the robot.

A useful tool to see which nodes are publishing and subscribing to a topic as well as what data type is being passed:

```
rostopic info [name of topic ex. wam/joint_states]
```

Some of the topics published and subscribed to by the Guardian-WAM are:

wam/joint\_states - Publishes the current joint angles, effort, and velocity of all the WAM joints.

wam/cart\_vel\_cmd - The WAM subscribes to real-time cartesian velocity commands.

guardian/command - The Guardian subscribes to linear and angular velocity commands.

To subscribe to a topic and print its values via the command line:

```
rostopic echo [name of topic ex. wam/joint_states]
```

This example will stream the current joint angles/effort/velocity of the WAM.

For more information on ROS Topics please visit:
[ROS Topics](http://www.ros.org/wiki/Topics) & [ROS Topics Tutorial](http://www.ros.org/wiki/ROS/Tutorials/UnderstandingTopics)


## Guardian-WAM Services ##

ROS Services use a Remote Procedure Call (RPC) for Request/Reply communication between nodes.  Services on the Guardian-WAM system expose a large amount of its functionality.

To view all of the rosservices on the ROS Graph in a terminal:
```
rosservice list
```

You will see a large list of services available for control of the robot.  Some of these include:

wam/joint\_move - for WAM Joint moves using trapezoidal motion control

wam/cart\_move -  for WAM Cartesian moves using trapezoidal motion control

also exposed is all of the functionality for the BarrettHand, a few are:

bhand/open\_grasp - for opening the BarrettHand

bhand/close\_spread -  for closing the BarrettHand spread

bhand/grasp\_pos - for moving the BarrettHand grasp to a specified position

and many, many more..

For more information on ROS Services please visit: [ROS Services](http://www.ros.org/wiki/Services) & [ROS Services Tutorial](http://www.ros.org/wiki/ROS/Tutorials/UnderstandingServicesParams)

The next tutorial will show us how to control the WAM using the services shown using the command line: **[Controlling the WAM with Service Calls.](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsWAMServiceCntrl)**