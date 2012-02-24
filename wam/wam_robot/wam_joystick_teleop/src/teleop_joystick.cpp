#include <vector>
#include <math.h>

#include <ros/ros.h>
#include "wam_msgs/BHandGraspVel.h"
#include "wam_msgs/BHandSpreadVel.h"
#include "wam_msgs/CartMove.h"
#include "wam_msgs/RTCart.h"
#include "wam_msgs/HoldJointPosition.h"
#include "std_srvs/Empty.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>

const int CNTRL_FREQ = 50;

class WamTeleop
{
public:
  ros::NodeHandle nh_;
  bool deadman, first_rt;
  bool grsp_publish, sprd_publish, cart_publish, home_publish, hold_publish;
  bool quat_publish, pose_pubbed, home_last, hold_last, home_st, hold_st, holding;
  int deadman_button, gpr_open_btn, gpr_close_btn;
  int sprd_open_btn, sprd_close_btn, bh_cmd_last;
  int quat_btn, home_btn, hold_btn;
  int axis_dx, axis_dy, axis_dz;
  double bh_max_gsp_vel, bh_max_sprd_vel;
  double bh_gsp_vel, bh_sprd_vel;
  double wam_max_dx, wam_max_dy, wam_max_dz;
  double req_cdx, req_cdy, req_cdz;
  std::vector<double> cart_cur, cart_last, cart_cmd;
  std::vector<double> quat_cur, quat_cmd;
  std::vector<double> rt_cart_cmd;
  wam_msgs::BHandGraspVel gsp_vel_srv;
  wam_msgs::BHandSpreadVel sprd_vel_srv;
  wam_msgs::CartMove cart_move_srv;
  wam_msgs::RTCart rt_position;
  wam_msgs::HoldJointPosition hold_jpos_srv;
  std_srvs::Empty go_home_srv;
  ros::Subscriber joy_sub;
  ros::Subscriber pose_sub;
  ros::ServiceClient grasp_vel;
  ros::ServiceClient spread_vel;
  ros::ServiceClient cart_move;
  ros::ServiceClient go_home;
  ros::ServiceClient hold_jpos;
  ros::Publisher rt_cart_pub;

  WamTeleop() :
      cart_cur(3), cart_last(3), cart_cmd(3), quat_cur(4), quat_cmd(4)
  {
  }

  void init()
  {
    nh_.param("deadman_button", deadman_button, 10);
    nh_.param("gripper_open_button", gpr_open_btn, 12);
    nh_.param("gripper_close_button", gpr_close_btn, 14);
    nh_.param("spread_open_button", sprd_open_btn, 13);
    nh_.param("spread_close_button", sprd_close_btn, 15);
    nh_.param("rotation_control_button", quat_btn, 10);
    nh_.param("go_home_button", home_btn, 0);
    nh_.param("hold_position_button", hold_btn, 3);
    nh_.param("grasp_max_velocity", bh_max_gsp_vel, 1.0);
    nh_.param("spread_max_velocity", bh_max_sprd_vel, 1.0);
    nh_.param("cartesian_x_delta", wam_max_dx, 0.008);
    nh_.param("cartesian_y_delta", wam_max_dy, 0.008);
    nh_.param("cartesian_z_delta", wam_max_dz, 0.008);
    nh_.param("cartesian_x_axis", axis_dx, 3);
    nh_.param("cartesian_y_axis", axis_dy, 2);
    nh_.param("cartesian_x_axis", axis_dz, 1);

    pose_pubbed = false; //bool to not send commands until wam pose is published.

    rt_position.position.resize(3);

    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &WamTeleop::joyCallback, this);
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("wam/pose", 1, &WamTeleop::poseCallback, this);
    grasp_vel = nh_.serviceClient<wam_msgs::BHandGraspVel>("bhand/grasp_vel");
    spread_vel = nh_.serviceClient<wam_msgs::BHandSpreadVel>("bhand/spread_vel");
    cart_move = nh_.serviceClient<wam_msgs::CartMove>("wam/cartesian_pos");
    rt_cart_pub = nh_.advertise<wam_msgs::RTCart>("wam/rtposition", 1);
    go_home = nh_.serviceClient<std_srvs::Empty>("wam/go_home");
    hold_jpos = nh_.serviceClient<wam_msgs::HoldJointPosition>("wam/hold_jpos");

    cart_move_srv.request.coordinates.resize(3);
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
  void send_commands();

  ~WamTeleop()
  {
  }
};

void WamTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  grsp_publish = sprd_publish = cart_publish = quat_publish = home_publish = hold_publish = home_last = hold_last =
      home_st = hold_st = holding = false;

  if (!joy_msg->buttons[deadman_button] || !pose_pubbed) // this returns with no deadman pressed or without first pose published yet.
    return;

  if (joy_msg->buttons[gpr_open_btn])
  {
    bh_gsp_vel = -bh_max_gsp_vel;
    grsp_publish = true;
  }
  else if (joy_msg->buttons[gpr_close_btn])
  {
    bh_gsp_vel = bh_max_gsp_vel;
    grsp_publish = true;
  }

  if (joy_msg->buttons[sprd_open_btn])
  {
    bh_sprd_vel = -bh_max_sprd_vel;
    sprd_publish = true;
  }
  else if (joy_msg->buttons[sprd_close_btn])
  {
    bh_sprd_vel = bh_max_sprd_vel;
    sprd_publish = true;
  }

  if (joy_msg->buttons[quat_btn])
  {
    quat_publish = true;
  }

  if ((joy_msg->axes[axis_dx] > 0.25 || joy_msg->axes[axis_dx] < -0.25))
  {
    req_cdx = -joy_msg->axes[axis_dx] * wam_max_dx;
    cart_publish = true;
  }
  else
    req_cdx = 0.0;
  if ((joy_msg->axes[axis_dy] > 0.25 || joy_msg->axes[axis_dy] < -0.25))
  {
    req_cdy = -joy_msg->axes[axis_dy] * wam_max_dy;
    cart_publish = true;
  }
  else
    req_cdy = 0.0;
  if ((joy_msg->axes[axis_dz] > 0.25 || joy_msg->axes[axis_dz] < -0.25))
  {
    req_cdz = joy_msg->axes[axis_dz] * wam_max_dz;
    cart_publish = true;
  }
  else
    req_cdz = 0.0;

  //For go_home command
  if (joy_msg->buttons[gpr_open_btn])
  {
    bh_gsp_vel = -bh_max_gsp_vel;
    grsp_publish = true;
  }
  else if (joy_msg->buttons[gpr_close_btn])
  {
    bh_gsp_vel = bh_max_gsp_vel;
    grsp_publish = true;
  }

  home_st = (joy_msg->buttons[home_btn]);
  if (home_st && !home_last)
  {
    home_publish = true;
  }
  else
  {
    home_publish = false;
  }
  home_last = home_publish;
  //for hold joint position command
  hold_st = (joy_msg->buttons[hold_btn]);
  if (hold_st && !hold_last)
  {
    hold_publish = true;
  }
  else
  {
    hold_publish = false;
  }

}

void WamTeleop::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  if (pose_pubbed == false)
  {
    cart_move_srv.request.coordinates[0] = pose_msg->pose.position.x;
    cart_move_srv.request.coordinates[1] = pose_msg->pose.position.y;
    cart_move_srv.request.coordinates[2] = pose_msg->pose.position.z;
  }
  pose_pubbed = true;
  cart_cur[0] = pose_msg->pose.position.x;
  cart_cur[1] = pose_msg->pose.position.y;
  cart_cur[2] = pose_msg->pose.position.z;
  quat_cur[0] = pose_msg->pose.orientation.x;
  quat_cur[1] = pose_msg->pose.orientation.y;
  quat_cur[2] = pose_msg->pose.orientation.z;
  quat_cur[3] = pose_msg->pose.orientation.w;
}

void WamTeleop::send_commands()
{

  if (grsp_publish && bh_cmd_last == 0 && !cart_publish)
  {
    gsp_vel_srv.request.velocity = bh_gsp_vel;
    grasp_vel.call(gsp_vel_srv);
    bh_cmd_last = 1;
  }
  if (sprd_publish && bh_cmd_last == 0 && !cart_publish)
  {
    sprd_vel_srv.request.velocity = bh_sprd_vel;
    spread_vel.call(sprd_vel_srv);
    bh_cmd_last = 2;
  }
  if (!grsp_publish && !sprd_publish && bh_cmd_last != 0 && !cart_publish)
  {
    if (bh_cmd_last == 1)
    {
      gsp_vel_srv.request.velocity = 0.0;
      grasp_vel.call(gsp_vel_srv);
    }
    else if (bh_cmd_last == 2)
    {
      sprd_vel_srv.request.velocity = 0.0;
      spread_vel.call(sprd_vel_srv);
    }
    bh_cmd_last = 0;
  }
  if (cart_publish && !grsp_publish && !sprd_publish && pose_pubbed)
  {
    rt_position.position[0] = cart_cur[0] + req_cdx;
    rt_position.position[1] = cart_cur[1] + req_cdy;
    rt_position.position[2] = cart_cur[2] + req_cdz;
    rt_cart_pub.publish(rt_position);
    cart_last[0] = rt_position.position[0];
    cart_last[1] = rt_position.position[1];
    cart_last[2] = rt_position.position[2];
    pose_pubbed = false;
  }

  if (hold_publish && !cart_publish && !grsp_publish && !sprd_publish && !home_publish)
  {
    if (!holding)
    {
      hold_jpos_srv.request.hold = true;
      holding = true;
    }
    else
    {
      hold_jpos_srv.request.hold = false;
      holding = false;
    }
    hold_jpos.call(hold_jpos_srv);
  }

  if(home_publish && !hold_publish && !cart_publish && !grsp_publish && !sprd_publish)
    go_home.call(go_home_srv);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wam_teleop");
  WamTeleop wam_teleop;
  wam_teleop.init();

  ros::Rate pub_rate(CNTRL_FREQ);

  while (wam_teleop.nh_.ok())
  {
    ros::spinOnce();
    wam_teleop.send_commands();
    pub_rate.sleep();
  }
  return 0;
}
