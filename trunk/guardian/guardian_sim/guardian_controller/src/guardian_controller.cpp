/*
 * guardian_controller
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
 * \brief Relate the 4-wheels joints with each motor, apply the control correction in closed loop for the motor and set the command received by the joystick/keyboard teleop node.
 */


#include "guardian_controller/guardian_controller.h"
#include <pluginlib/class_list_macros.h>

namespace guardian_controller_ns {


bool GuardianControllerClass::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{

  // 4-Axis Skid Steer Rover

  // back left wheel
  ROS_FATAL("GuardianController");
  std::string joint_blw;
  if (!n.getParam("joint_blw", joint_blw))
  {
    ROS_ERROR("No joint BLW given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }
  joint_state_blw_ = robot->getJointState(joint_blw);
  if (!joint_state_blw_)
  {
    ROS_ERROR("GuardianController could not find joint named '%s'",
              joint_blw.c_str());
    return false;
  }
  ROS_DEBUG("GuardianController found joint named '%s'", joint_blw.c_str());

  // front left wheel
  std::string joint_flw;
  if (!n.getParam("joint_flw", joint_flw))
  {
    ROS_ERROR("No joint FLW given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }
  joint_state_flw_ = robot->getJointState(joint_flw);
  if (!joint_state_flw_)
  {
    ROS_ERROR("GuardianController could not find joint named '%s'",
              joint_flw.c_str());
    return false;
  }
  ROS_DEBUG("GuardianController found joint named '%s'", joint_flw.c_str());

  // back right wheel
  std::string joint_brw;
  if (!n.getParam("joint_brw", joint_brw))
  {
    ROS_ERROR("No joint BRW given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }
  joint_state_brw_ = robot->getJointState(joint_brw);
  if (!joint_state_brw_)
  {
    ROS_ERROR("GuardianController could not find joint named '%s'",
              joint_brw.c_str());
    return false;
  }
  ROS_DEBUG("GuardianController found joint named '%s'", joint_brw.c_str());

  // front right wheel
  std::string joint_frw;
  if (!n.getParam("joint_frw", joint_frw))
  {
    ROS_ERROR("No joint FRW given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }
  joint_state_frw_ = robot->getJointState(joint_frw);
  if (!joint_state_frw_)
  {
    ROS_ERROR("GuardianController could not find joint named '%s'",
              joint_frw.c_str());
    return false;
  }
  //ROS_ERROR("GuardianController found joint named '%s'", joint_frw.c_str());

  // Robot Speeds
  linearSpeedMps_   = 0.0;
  angularSpeedRads_ = 0.0;

  // Robot Positions
  robot_pose_px_ = 0.0;
  robot_pose_py_ = 0.0;
  robot_pose_pa_ = 0.0;

  // External speed references
  v_ref_ = 0.0;
  w_ref_ = 0.0;

  // Frequency
  frequency = 100.0; // Hz 50.0

  // for later access
  node_ = n;

  cmd_sub_ = node_.subscribe<geometry_msgs::Twist>("command", 1, &GuardianControllerClass::commandCallback, this);

  return true;
}


/// Controller startup in realtime
void GuardianControllerClass::starting()
{
  // Initial positions
  init_pos_blw_ = joint_state_blw_->position_;
  init_pos_flw_ = joint_state_flw_->position_;
  init_pos_brw_ = joint_state_brw_->position_;
  init_pos_frw_ = joint_state_frw_->position_;
  // Initial velocities
  init_vel_blw_ = joint_state_blw_->velocity_;
  init_vel_flw_ = joint_state_flw_->velocity_;
  init_vel_brw_ = joint_state_brw_->velocity_;
  init_vel_frw_ = joint_state_frw_->velocity_;

}


/// Controller update loop in realtime
void GuardianControllerClass::update()
{

  double v_left_mps, v_right_mps;

  // Calculate its own velocities for realize the motor control
  v_left_mps = ((joint_state_blw_->velocity_ + joint_state_flw_->velocity_) / 2.0) * (GUARDIAN_WHEEL_DIAMETER / 2.0);
  v_right_mps =((joint_state_brw_->velocity_ + joint_state_frw_->velocity_) / 2.0) * (GUARDIAN_WHEEL_DIAMETER / 2.0);

  linearSpeedMps_ = (v_right_mps + v_left_mps) / 2.0;                      // m/s
  angularSpeedRads_ = (v_right_mps - v_left_mps) / GUARDIAN_D_TRACKS_M;    //rad/s

  // ControlVW 
  // Note : this state space control was implemented for gazebo simulation (and old AX controllers)

  // Current AX3500 controllers close this loop allowing (v,w) references.
  double epv=0.0;
  double epw=0.0;
  static double epvant =0.0;
  static double epwant =0.0;

  // Adjusted for soft indoor office soil
  double kpv=100.0; double kdv=0.0;
  double kpw=200.0;  double kdw=0.0;

  // State feedback error
  epv = v_ref_ - linearSpeedMps_;
  epw = w_ref_ - angularSpeedRads_;

  // Compute state control actions
  double uv= kpv * epv + kdv * (epv - epvant);
  double uw= kpw * epw + kdw * (epw - epwant);
  epvant = epv;
  epwant = epw;

  // Inverse kinematics
  double dUl = uv - 0.5 * GUARDIAN_D_TRACKS_M * uw;
  double dUr = uv + 0.5 * GUARDIAN_D_TRACKS_M * uw;

  // Motor control actions
  double limit = 40.0;
    
  if(left_right==0){
    //ROS_INFO("Forward/Backward or rotating over itself");
    joint_state_blw_->commanded_effort_ = saturation(-10.0 * (joint_state_blw_->velocity_ - dUl), -limit, limit);
    joint_state_flw_->commanded_effort_ = saturation(-10.0 * (joint_state_flw_->velocity_ - dUl), -limit, limit);
    joint_state_brw_->commanded_effort_ = saturation(-10.0 * (joint_state_brw_->velocity_ - dUr), -limit, limit);
    joint_state_frw_->commanded_effort_ = saturation(-10.0 * (joint_state_frw_->velocity_ - dUr), -limit, limit);
    }else if (left_right==-1){
    //ROS_INFO("Turning left");
    joint_state_brw_->commanded_effort_ = saturation(-10.0 * (joint_state_brw_->velocity_ - dUr), -limit, limit);
    joint_state_frw_->commanded_effort_ = saturation(-10.0 * (joint_state_frw_->velocity_ - dUr), -limit, limit);
    }else if (left_right==1){
    //ROS_INFO("Turning right");
    joint_state_blw_->commanded_effort_ = saturation(-10.0 * (joint_state_blw_->velocity_ - dUl), -limit, limit);
    joint_state_flw_->commanded_effort_ = saturation(-10.0 * (joint_state_flw_->velocity_ - dUl), -limit, limit);
    }else{
    //ROS_INFO("Stoped or rotating over itself");
    joint_state_blw_->commanded_effort_ = saturation(-10.0 * (joint_state_blw_->velocity_ - dUl), -limit, limit);
    joint_state_flw_->commanded_effort_ = saturation(-10.0 * (joint_state_flw_->velocity_ - dUl), -limit, limit);
    joint_state_brw_->commanded_effort_ = saturation(-10.0 * (joint_state_brw_->velocity_ - dUr), -limit, limit);
    joint_state_frw_->commanded_effort_ = saturation(-10.0 * (joint_state_frw_->velocity_ - dUr), -limit, limit);
   }
}


/// Controller stopping in realtime
void GuardianControllerClass::stopping()
{}


// Set the base velocity command
void GuardianControllerClass::setCommand(const geometry_msgs::Twist &cmd_vel)
{
     // left_right

     //  0: Moving forward/backward
     // -1: Turning left
     // +1: Turning right

     if(cmd_vel.linear.x!=0){	
     	v_ref_ = saturation(cmd_vel.linear.x, -10.0, 10.0);
	left_right=0;
     }else if(cmd_vel.linear.y<0){
     	v_ref_ = -1 * saturation(cmd_vel.linear.y, -10.0, 10.0); 
	left_right=1; 						
     }else if(cmd_vel.linear.y>0){
	v_ref_ = saturation(cmd_vel.linear.y, -10.0, 10.0);
	left_right=-1;
     }else{
	v_ref_= 0;
	left_right=-2; // Any other key will be interpretated as a stop order
     }
    
     w_ref_ = saturation(cmd_vel.angular.z, -1.0, 1.0); // -10.0 10.0

}


void GuardianControllerClass::commandCallback(const geometry_msgs::TwistConstPtr& msg)
{
  base_vel_msg_ = *msg;
  this->setCommand(base_vel_msg_);
}


double GuardianControllerClass::saturation(double u, double min, double max)
{
 if (u>max) u=max;
 if (u<min) u=min;
 return u; 
}

} // namespace


/// Register controller to pluginlib

PLUGINLIB_DECLARE_CLASS(guardian_controller, GuardianControllerPlugin,
                         guardian_controller_ns::GuardianControllerClass,
                         pr2_controller_interface::Controller)

