# Guardian-WAM Tutorial: Writing a Guardian-WAM Control Program #

**Now that we have a understanding of controlling the WAM and BarrettHand with ROS Service calls from the previous two tutorials:**

[Controlling the WAM with Service Calls](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsWAMServiceCntrl)

[Controlling the BarrettHand with Service Calls](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsBHANDCntrl)

**We will use that knowledge and understanding to create our own standalone Guardian-WAM control program.**

This portion of the tutorial _**does not**_ require the Guardian-WAM to be on and running.

## Instructions ##

On a user PC, with a network connection to the Guardian-WAM.

Enter into our user created path for ROS Packages:
```
cd ~/ros
```

You can verify that your ~/ros folder is in the ROS Package Path with
```
env | grep ROS_PACKAGE_PATH
```
You should see ~/ros somewhere in the path if this is your working ROS directory

In your ros directory (~/ros) workspace.

Create a new ROS package
```
roscreate-pkg gwam_tutorials

#Enter your new ROS Package
cd gwam_tutorials

#Make a folder for the source code we will write
mkdir src

#Create the c++ file that we will write to control the Guardian-WAM
#For this we will use Gedit
gedit src/gwam_control_tutorial.cpp
```

Copy and Paste the following source code into gwam\_control\_tutorial.cpp
```
/** Guardian-WAM Control Program Publishing Guardian Velocity  **
*** Messages and WAM / BarrettHand Service Calls.              **/

//Include our main ros directive
#include <ros/ros.h>

//Include or WAM specific services 
#include "wam_srvs/BHandFingerVel.h"
#include "wam_srvs/JointMove.h"

//Include empty messages for opening the BarrettHand Spread and Moving the WAM Home
#include "std_srvs/Empty.h"
//Including geometry twist messages to control the Guardian Base
#include "geometry_msgs/Twist.h"


int main(int argc, char** argv)
{
  //Initialize or ROS node 
  ros::init(argc, argv, "gwam_tutorial_node");
  
  //Create a ROS Nodehandle for our publisher and clients
  ros::NodeHandle n;

  /* WAM Control Portion */
  
  //Initialize our service clients to the specified topics
  ros::ServiceClient bhand_sc_clnt = n.serviceClient<std_srvs::Empty>("bhand/close_spread");
  ros::ServiceClient bhand_fvel_clnt = n.serviceClient<wam_srvs::BHandFingerVel>("bhand/finger_vel");
  ros::ServiceClient wam_jmove_clnt = n.serviceClient<wam_srvs::JointMove>("wam/joint_move");
  ros::ServiceClient wam_home_clnt = n.serviceClient<std_srvs::Empty>("wam/go_home");

  // Declare our services to be called
  std_srvs::Empty close_sprd;
  std_srvs::Empty go_home;  
  wam_srvs::BHandFingerVel fvel_srv;
  wam_srvs::JointMove jmove_srv;

  // Call the BarrettHand close spread service call with an empty service
  bhand_sc_clnt.call(close_sprd);

  ros::Duration(3.0).sleep(); // The BarrettHand will close the spread and wait for 3 seconds

  // Fill in velocities to control the individual finger velocities
  fvel_srv.request.velocity[0] = 0.25;
  fvel_srv.request.velocity[1] = 0.5;
  fvel_srv.request.velocity[2] = 1.0;

  // Call the BarrettHand finger velocity service
  bhand_fvel_clnt.call(fvel_srv);
  
  ros::Duration(5.0).sleep(); // The BarrettHand will move the fingers at the velocity specified and wait 3 seconds

  // We must first resize our joint vector to reflect our 7-DOF WAM
  jmove_srv.request.joints.resize(7);

  // We can now give joint positions in radians to each joint (Radians)
  jmove_srv.request.joints[0] = 0;
  jmove_srv.request.joints[1] = 0;
  jmove_srv.request.joints[2] = 0;
  jmove_srv.request.joints[3] = 1.57;
  jmove_srv.request.joints[4] = 0;
  jmove_srv.request.joints[5] = 0;
  jmove_srv.request.joints[6] = -1.57;
  
  // Call the WAM joint move service to reflect the desired joint positions
  wam_jmove_clnt.call(jmove_srv);

  ros::Duration(8.0).sleep(); // The WAM will move to the commanded joint position and wait for 3 seconds
  
  //Lastly we will call the WAM go_home service to send the WAM to its home position.
  wam_home_clnt.call(go_home);

  ros::Duration(3.0).sleep(); // The WAM will go to its zeroed joint positions in order to avoid collisions with the WAM and gently set itself back home

  /* Guardian Base Control Portion */  

  // Initialize our publisher to publish on the guardian_node/command topic
  ros::Publisher guardian_move_pub = n.advertise<geometry_msgs::Twist>("guardian_node/command",1);
  ros::Duration(2.0).sleep();
 

  // Declare our guardian message
  geometry_msgs::Twist guardian_msg;
  

  //Fill in the angular velocities to make the guardian spin in place
  guardian_msg.linear.x = 0.0;
  guardian_msg.linear.y = 0.0;
  guardian_msg.linear.z = 0.0;

  guardian_msg.angular.x = 0.1;
  guardian_msg.angular.y = 0.1;
  guardian_msg.angular.z = 0.1;
  
  // Publish the message to our topic, the Guardian Node is subscribing and will move the Guardian accordingly
  guardian_move_pub.publish(guardian_msg);

  ros::Duration(4.0).sleep(); // The guardian will rotate for two seconds

  // Give new angular commands to spin the Guardian the opposite direction.
  guardian_msg.angular.x = -0.1;
  guardian_msg.angular.y = -0.1;
  guardian_msg.angular.z = -0.1;

  guardian_move_pub.publish(guardian_msg);

  ros::Duration(4.0).sleep(); // The guardian will rotate the opposite direction for two seconds
  
  // Update the angular commands to stop the Guardian.
  guardian_msg.angular.x = 0.0;
  guardian_msg.angular.y = 0.0;
  guardian_msg.angular.z = 0.0;
  
  guardian_move_pub.publish(guardian_msg);

  ros::Duration(1.0).sleep(); // The guardian stop and give the program one second before continuing 
  
  return 0;
}
```
Save and close the file.


Now that we have created our file and understand how to make service calls from our own control program.  We will learn how to compile and run the program.

Edit the manifest.xml in order for our program to retrieve the necessary dependencies.

Please make your manifest.xml replicate the file below
```
<package>
  <description brief="gwam_tutorials">

     Creating my first Guardian-WAM Control Program!

  </description>
  <author>me</author>
  <license>BSD</license>
  <review status="unreviewed" notes=""/>
  <url>http://ros.org/wiki/gwam_tutorials</url>
  <depend package="roscpp"/>
  <depend package="std_srvs"/>
  <depend package="wam_srvs"/>
  <depend package="geometry_msgs"/>
</package>
```
Save and Exit.

Lastly, we need to edit our CMakeLists file in order to tell the compiler to create an executable from our source file.

Please make your CMakeLists.txt file replicate the file below
```
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(gwam_control_tutorial src/gwam_control_tutorial.cpp)
```

Save and Exit.

We are now ready to compile our program
```
rosmake
```


### Running Your Program on the Guardian-WAM ###
Running this code _**REQUIRES**_ a 7-DOF WAM and BarrettHand

**Please bring up the entire Guardian-WAM System before continuing with this tutorial**

> - Turn on the Guardian-WAM Master Power, WAM power, start the Guardian PC, and reset the power to the motor controller.

> - Once the Guardian system is online, please SSH into the WAM PC and launch the wam\_node

```
ssh robot@WAM
#password: WAM

roslaunch wam_bringup wam_guardian.launch
```

Your system should now be initialized and running, please continue with the tutorial.

On your local machine where you created your Guardian-WAM Control Program:
```
roscd gwam_tutorials

#Start your program with the following command
rosrun gwam_tutorials gwam_control_tutorial
```

Congratulations you have just written your first Guardian-WAM Control Program!

Now that we have a full understanding how to create our own Guardian-wAM control program lets take a look at a more complicated example of this:  [Understanding the Guardian-WAM Joystick Teleoperation](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsGWAMJoystickTeleop)