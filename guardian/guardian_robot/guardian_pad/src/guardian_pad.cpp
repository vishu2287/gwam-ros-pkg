/*
 * guardian_pad
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
 * \brief Allows to use a pad with the guardian_controller, sending the messages received through the joystick input, correctly adapted, to the "guardian_controller/command" by default.
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

//#include <modbus_io/modbus_io.h>
#include <modbus_io/write_digital_output.h>

////////////////////////////////////////////////////////////////////////
//                               NOTE:                                //
// This configuration is made for a THRUSTMASTER T-Wireless 3in1 Joy  //
//   please feel free to modify to adapt for your own joystick.       //   
// 								      //
////////////////////////////////////////////////////////////////////////


class GuardianPad
{
public:
  GuardianPad();

private:
  void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber pad_sub_;
  std::string cmd_topic_;
  double current_vel;
  bool sim_mode_;
  int dead_man_button_;
  int speed_up_button_, speed_down_button_;
  int button_output_1_, button_output_2_;
  int output_1_, output_2_;

  ros::ServiceClient modbus_write_do_client;  
};


GuardianPad::GuardianPad():
  linear_(1),
  angular_(2)
{

  current_vel = 0.1;

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("cmd_topic", cmd_topic_, cmd_topic_);
  nh_.param("dead_man_button", dead_man_button_, dead_man_button_);
  nh_.param("speed_up_button", speed_up_button_, speed_up_button_);  //4 Thrustmaster
  nh_.param("speed_down_button", speed_down_button_, speed_down_button_); //5 Thrustmaster
  nh_.param("button_output_1", button_output_1_, button_output_1_);
  nh_.param("button_output_2", button_output_2_, button_output_2_);
  nh_.param("output_1", output_1_, output_1_);
  nh_.param("output_2", output_2_, output_2_);

  // Publish through the node handle Twist type messages to the guardian_controller/command topic
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);

  // Listen through the node handle sensor_msgs::Joy messages from joystick (these are the orders that we will send to guardian_controller/command)
  pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &GuardianPad::padCallback, this);

  // Request service to activate / deactivate digital I/O
  modbus_write_do_client = nh_.serviceClient<modbus_io::write_digital_output>("/modbous_io_node/write_digital_output");

}

void GuardianPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  
  // Actions dependant on dead-man button
  if (joy->buttons[dead_man_button_] == 1) {
	// Set the current velocity level
	if (joy->buttons[speed_up_button_] == 1 && current_vel > 0.1){
	  current_vel = current_vel - 0.1;
	  ROS_ERROR("Velocity: %f%%", current_vel*100.0);
	}else if (joy->buttons[speed_down_button_] == 1 && current_vel < 0.9){
	  current_vel = current_vel + 0.1;
	  ROS_ERROR("Velocity: %f%%", current_vel*100.0);
	}
	vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
	vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
	vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
	vel.linear.x = current_vel*l_scale_*joy->axes[linear_];
	vel.linear.y = current_vel*l_scale_*joy->axes[linear_];
	vel.linear.z = current_vel*l_scale_*joy->axes[linear_];

	if (joy->buttons[button_output_1_] == 1) {
		modbus_io::write_digital_output modbus_wdo_srv;
		modbus_wdo_srv.request.output = output_1_;
		modbus_wdo_srv.request.value = true;
		modbus_write_do_client.call( modbus_wdo_srv );
		}
	else {
	     }
	}
   else {
	vel.angular.x = 0.0;	vel.angular.y = 0.0; vel.angular.z = 0.0;
	vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
	}

   vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guardian_pad");
  GuardianPad guardian_pad;
  ros::spin();
}

