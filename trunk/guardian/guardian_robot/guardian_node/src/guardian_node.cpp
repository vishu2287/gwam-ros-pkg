/*
 * guardian_node
 * Copyright (c) 2011, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 */

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <guardian_hardware_interface/guardian_hardware_interface.h>
#include <string>
#include <ros/ros.h>

#define 	frequency 	50.0

//////////////////////////
// Variable definitions //
//////////////////////////

std::string port;

guardian_hw_iface::guardian_hardware_interface * guardian; 		// Instance to the interface


double v, w = 0.0; 							// Callback cmd vel variables

double vx, vy, va, px, py, pa, dt = 0.0;				// Odometry variables

int token = 0;

// Recovery attemps control
int maxAttemps = 10000;
int attemps = 0;

// IMU values
double ang_vel_x_ = 0.0;
double ang_vel_y_ = 0.0;
double ang_vel_z_ = 0.0;

double lin_acc_x_ = 0.0;
double lin_acc_y_ = 0.0;
double lin_acc_z_ = 0.0;

double orientation_x_ = 0.0;
double orientation_y_ = 0.0;
double orientation_z_ = 0.0;
double orientation_w_ = 0.0;


//////////////////////////
// Callback definitions //
//////////////////////////

void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
	// Multiplied * -1 to adapt the real direction with the joystick commands
	v = cmd_vel->linear.x * -1;
	w = cmd_vel->angular.z * -1;					
	guardian->SetRobotSpeed(v, w);
}


void imuCallback(const sensor_msgs::Imu& imu_msg){
	orientation_x_ = imu_msg.orientation.x;
	orientation_y_ = imu_msg.orientation.y;
	orientation_z_ = imu_msg.orientation.z;
	orientation_w_ = imu_msg.orientation.w;

	ang_vel_x_ = imu_msg.angular_velocity.x;
	ang_vel_y_ = imu_msg.angular_velocity.y;
	ang_vel_z_ = imu_msg.angular_velocity.z;

	lin_acc_x_ = imu_msg.linear_acceleration.x;
	lin_acc_y_ = imu_msg.linear_acceleration.y;
	lin_acc_z_ = imu_msg.linear_acceleration.z;
}


void ChangeRefFrame(double *vx, double *vy, double *va, double *px, double *py, double *pa){
	*vx = *vx * (-1.0);
	*vy = *vy * (-1.0);
	*va = *va * (-1.0);
	*px = *px * (-1.0);
	*py = *py * (-1.0);
	*pa = *pa * (-1.0);
}


//////////
// main //
//////////

int main(int argc, char** argv)
{

	ros::init(argc, argv, "guardian_node");
	ros::NodeHandle n;

	ROS_INFO("Guardian for ROS.");
	// set the device port, reading the configuration
	std::string sDevicePort;
	std::string sDefaultPort(GUARDIAN_HW_IFACE_DEFAULT_PORT);
	n.param("motor_dev", sDevicePort,  sDefaultPort);
	printf("guardian_node::main: Motor dev = %s\n", sDevicePort.c_str());

	// Interface creation (Closed Loop Mixed Velocity Control)
	guardian = new guardian_hw_iface::guardian_hardware_interface(50.0, sDevicePort.c_str());									

  	if(!guardian){
      		ROS_ERROR("Something wrong with the hardware interface pointer!");
  	}else{
		ROS_INFO("Port opened successfully");
	}

	// Define subscribers to obtain information through the sensors and joysticks
  	ros::Subscriber cmd_sub_ = n.subscribe<geometry_msgs::Twist>("/guardian_node/command", 1, cmdCallback); 
  	ros::Subscriber imu_sub_ = n.subscribe("/imu/data", 1, imuCallback);

	// Define publishers
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(10.0);  // 50.0 
	
	while( attemps < maxAttemps ) {

		while( ros::ok() ){ // n.ok() && (guardian->GetCurrentState() == 1)

			current_time = ros::Time::now();

			// + 1 due to the first ReadyState call is made by InitState, next to the creation of the instance object
			guardian->ReadyState(token+1);	

			if (token < 3) token++;
			else token = -1;

			// Obtain the odometry 
			guardian->GetOdometry(&vx, &vy, &va, &px, &py, &pa);

			// Adapt the odometry to the new reference
			ChangeRefFrame(&vx, &vy, &va, &px, &py, &pa);
		
			//ROS_WARN("*********************************\n");
			//ROS_INFO("Vx: %d Vy: %d Va: %d\n", vx, vy, va);
			//ROS_INFO("Px: %d Py: %d Pa: %d\n", px, py, pa);
			//ROS_INFO("Temp: %d\n", guardian->GetTemp(1)) ;
			//ROS_INFO("Voltage: %d\n", guardian->GetVoltage());
			//ROS_INFO("GetEncoder(L): %f\n", guardian->GetEncoder('L') );
			//ROS_INFO("GetEncoder(R): %f\n", guardian->GetEncoder('R') );
			//ROS_WARN("*********************************\n");
		
			dt = (current_time - last_time).toSec();

			//since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pa);
		
			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_footprint"; // base_link

			odom_trans.transform.translation.x = px;
			odom_trans.transform.translation.y = py;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = px;
			odom.pose.pose.position.y = py;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			double yfq = tf::getYaw(odom_quat);
			//ROS_ERROR("EULER YAW FROM QUATERNION:%f", yfq);
		
			//set the velocity
			odom.child_frame_id = "base_footprint"; // base_link
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = va;
		
			//publish the message
			odom_pub.publish(odom);

			last_time = current_time;

			ros::spinOnce();
			r.sleep();
		}
		
		ROS_WARN("guardian_hardware_interface::main bucle: Trying to recover the ready state.");

		attemps = attemps + 1;

	}

	ROS_ERROR("guardian_hardware_interface::main bucle: Unable to recover the control. Powering off the robot.");
	guardian->ToggleMotorPower('0');
	// Destroy the instance
	guardian->~guardian_hardware_interface();

}


