/*
* jt_example.cpp
* June 6, 2012
* Authore: Kyle Maroney
*/

// This program will publish an oscillating joint torque for J2

#include <math.h>
#include <time.h>
#include "ros/ros.h"
#include "wam_msgs/RTJointTrq.h"
#include "wam_srvs/Hold.h"

// You may need to change the amplitude to see the correct results if 
// There is an BHand / End Effector Present
const double amp = 4.0; //Nm
const int pub_rate = 100; //Hz
const double freq = .01; //rad/s * Hz

//Globals
wam_msgs::RTJointTrq jt_cmd;
double theta;

void update(void)
{
  theta += freq;
  jt_cmd.torques[1] = amp * std::sin(theta);
}

int main(int argc, char **argv)
{
  // Initialize our node
  ros::init(argc, argv, "jt_example_node");

  // Giving ourselves a print out to display node successfully started
  ROS_INFO("JT Example Program Started");
  
  // Set up a nodeHandle 
  ros::NodeHandle n;

  // Register our publisher
  ros::Publisher jt_pub = n.advertise<wam_msgs::RTJointTrq>("wam/jnt_trq_cmd", 1);

  // By default when the BarrettHand is present joint hold defaults on 
  // during wam_node startup.  We will release the joint hold.
  wam_srvs::Hold hold;
  ros::ServiceClient jp_hold = n.serviceClient<wam_srvs::Hold>("wam/hold_joint_pos");
  jp_hold.call(hold);

  // Set publishing rate - 100 Hz
  ros::Rate loop_rate(pub_rate);

  // Initialize our JT Command
  jt_cmd.torques.resize(7); // Resize message to DOF of WAM desired

  for(int i = 0; i < 7; i++)
    jt_cmd.torques[i] = 0.0;

  theta = 0.0;  

  while (ros::ok())
  {
    // Update new torque command
    update();

    // Publish our newly calculated torque
    jt_pub.publish(jt_cmd);

    ros::spinOnce();

    // Sleep for the specified publishing rate
    loop_rate.sleep();
  }


  return 0;
}
