#include <unistd.h>
#include <math.h>

#include "ros/ros.h"
#include "wam_msgs/GravityComp.h"
#include "wam_msgs/JointMove.h"
#include "wam_msgs/CartMove.h"
#include "wam_msgs/HoldPose.h"
#include "wam_msgs/HoldJointPosition.h"
#include "wam_msgs/BHandGraspPos.h"
#include "wam_msgs/BHandSpreadPos.h"
#include "wam_msgs/BHandGraspVel.h"
#include "wam_msgs/BHandSpreadVel.h"
#include "wam_msgs/RTCart.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include <barrett/math.h> 
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>

static const int PUBLISH_FREQ = 60;

using namespace barrett;
//using systems::connect;

bool newpos = false;
bool rt_status = false;
bool grav_state = true;
bool holding_pose = false;
bool holding_j_pos = false;
bool got_hand = false;
int wam_dof = 0;
systems::Wam<4>* wam4 = NULL;
systems::Wam<7>* wam7 = NULL;
Hand* hand = NULL;
units::JointPositions<4>::type* jp4;
units::JointPositions<4>::type* jp4c;
units::JointPositions<4>::type* jphnd;
units::JointPositions<7>::type* jp7;
units::JointPositions<7>::type* jp7c;
units::JointTorques<4>::type* jt4;
units::JointTorques<7>::type* jt7;
units::JointVelocities<4>::type* jv4;
units::JointVelocities<4>::type* jvhnd;
units::JointVelocities<7>::type* jv7;
units::CartesianPosition::type* cpc;
units::CartesianPosition::type* cprtc;
Eigen::Quaterniond toc;
systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint;
systems::ExposedOutput<units::CartesianPosition::type> positionSetPoint;

std::vector<double> rt_pos_cmd;

template<size_t DOF>
  systems::Wam<DOF>* getWam(ProductManager& pm)
  {
    assert(false);
    return NULL;
  }
template<>
  systems::Wam<7>* getWam(ProductManager& pm)
  {
    return pm.getWam7();
  }
template<>
  systems::Wam<4>* getWam(ProductManager& pm)
  {
    return pm.getWam4();
  }

class WamNode
{
protected:
  sensor_msgs::JointState wam_joint_state;
  sensor_msgs::JointState bhand_joint_state;
  geometry_msgs::PoseStamped wam_pose;
  ros::Publisher wam_joint_pub;
  ros::Publisher bhand_joint_pub;
  ros::Publisher pose_pub;
  ros::Subscriber rt_cart_sub;
  ros::ServiceServer gravity_srv;
  ros::ServiceServer jmove_srv;
  ros::ServiceServer bhand_sp_pos_srv;
  ros::ServiceServer bhand_gsp_pos_srv;
  ros::ServiceServer bhand_sp_vel_srv;
  ros::ServiceServer bhand_gsp_vel_srv;
  ros::ServiceServer cmove_srv;
  ros::ServiceServer hold_pose_srv;
  ros::ServiceServer hold_j_position_srv;

  ros::Time last_recieved_rt_pos_msg_time;
  ros::Duration rt_pos_msg_timeout;

public:
  WamNode()
  {
  }

  template<size_t DOF>
    void init(ProductManager& pm);

  ~WamNode()
  {
    delete cpc;
    delete cprtc;
    delete jp4;
    delete jp4c;
    delete jphnd;
    delete jt4;
    delete jv4;
    delete jvhnd;
    delete jp7;
    delete jt7;
    delete jv7;
  }

  bool gravity(wam_msgs::GravityComp::Request &req, wam_msgs::GravityComp::Response &res);
  bool joint_move(wam_msgs::JointMove::Request &req, wam_msgs::JointMove::Response &res);
  bool cart_move(wam_msgs::CartMove::Request &req, wam_msgs::CartMove::Response &res);
  void rt_cart_cb(const wam_msgs::RTCart::ConstPtr& rtmove);
  bool hold_pose(wam_msgs::HoldPose::Request &req, wam_msgs::HoldPose::Response &res);
  bool hold_joint_position(wam_msgs::HoldJointPosition::Request &req, wam_msgs::HoldJointPosition::Response &res);
  bool bhand_spread_pos(wam_msgs::BHandSpreadPos::Request &req, wam_msgs::BHandSpreadPos::Response &res);
  bool bhand_grasp_pos(wam_msgs::BHandGraspPos::Request &req, wam_msgs::BHandGraspPos::Response &res);
  bool bhand_spread_vel(wam_msgs::BHandSpreadVel::Request &req, wam_msgs::BHandSpreadVel::Response &res);
  bool bhand_grasp_vel(wam_msgs::BHandGraspVel::Request &req, wam_msgs::BHandGraspVel::Response &res);
  template<size_t DOF>
    void update_publisher(ProductManager& pm);

};

template<size_t DOF>
  void WamNode::init(ProductManager& pm)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    systems::Wam < DOF > &wam = *getWam<DOF>(pm);

    ros::NodeHandle n_("wam");
    ros::NodeHandle nh_("bhand");
    if (pm.foundWam4())
    {
      wam4 = pm.getWam4();
      wam_dof = 4;
      jp4 = new units::JointPositions<4>::type;
      jp4c = new units::JointPositions<4>::type;
      jt4 = new units::JointTorques<4>::type;
      jv4 = new units::JointVelocities<4>::type;
    }
    else if (pm.foundWam7())
    {
      wam7 = pm.getWam7();
      wam_dof = 7;
      jp7 = new units::JointPositions<7>::type;
      jp7c = new units::JointPositions<7>::type;
      jt7 = new units::JointTorques<7>::type;
      jv7 = new units::JointVelocities<7>::type;
    }
    cpc = new units::CartesianPosition::type;
    cprtc = new units::CartesianPosition::type;

    if (pm.foundHand())
    {
      jp_type jpt = wam.getJointPositions();
      jpt[3] = jpt[3] - 0.5;
      wam.moveTo(jpt);
      hand = pm.getHand();
      jphnd = new units::JointPositions<4>::type;
      jvhnd = new units::JointVelocities<4>::type;
      got_hand = true;
    }

    gravity_srv = n_.advertiseService("gravity_comp", &WamNode::gravity, this);
    jmove_srv = n_.advertiseService("joint_pos", &WamNode::joint_move, this);
    cmove_srv = n_.advertiseService("cartesian_pos", &WamNode::cart_move, this);
    hold_pose_srv = n_.advertiseService("orientation_hold", &WamNode::hold_pose, this);
    hold_j_position_srv = n_.advertiseService("joint_pos_hold", &WamNode::hold_joint_position, this);
    wam_joint_pub = n_.advertise<sensor_msgs::JointState>("joints", 100);
    pose_pub = n_.advertise<geometry_msgs::PoseStamped>("pose", 100);
    rt_cart_sub = n_.subscribe("rtposition", 1, &WamNode::rt_cart_cb, this);

    const char* strarray[] = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
    std::vector<std::string> wam_joints(strarray, strarray + 7);
    wam_joint_state.name = wam_joints;
    wam_joint_state.name.resize(wam_dof);
    wam_joint_state.position.resize(wam_dof);
    wam_joint_state.velocity.resize(wam_dof);
    wam_joint_state.effort.resize(wam_dof);

    if (got_hand) //If BarrettHand Found on Bus, Initialize hand and begin advertising joint messages and control services
    {
      hand->initialize();
      hand->updatePosition(true);
      hand->updatePosition();
      hand->update(Hand::S_ALL, true);
      hand->update();
      bhand_joint_pub = nh_.advertise<sensor_msgs::JointState>("bhand_joints", 100);
      bhand_sp_pos_srv = nh_.advertiseService("spread_pos", &WamNode::bhand_spread_pos, this);
      bhand_gsp_pos_srv = nh_.advertiseService("grasp_pos", &WamNode::bhand_grasp_pos, this);
      bhand_sp_vel_srv = nh_.advertiseService("spread_vel", &WamNode::bhand_spread_vel, this);
      bhand_gsp_vel_srv = nh_.advertiseService("grasp_vel", &WamNode::bhand_grasp_vel, this);

      const char* strarray[] = {"inner1", "inner2", "inner3", "spread", "outer1", "outer2", "outer3"};
      std::vector<std::string> hand_joints(strarray, strarray + 7);
      hand_joints.resize(7);
      bhand_joint_state.name.resize(7);
      bhand_joint_state.name = hand_joints;
      bhand_joint_state.position.resize(7);
    }

    //Setting up the RT Cart Move Timeout
    rt_pos_msg_timeout.fromSec(0.2);

    //resizing rt position command
    rt_pos_cmd.resize(3);
  }

void WamNode::rt_cart_cb(const wam_msgs::RTCart::ConstPtr& msg)
{
  if (rt_status)
  {
    rt_pos_cmd[0] = msg->position[0];
    rt_pos_cmd[1] = msg->position[1];
    rt_pos_cmd[2] = msg->position[2];
    newpos = true;
  }
  last_recieved_rt_pos_msg_time = ros::Time::now();
}

bool WamNode::hold_pose(wam_msgs::HoldPose::Request &req, wam_msgs::HoldPose::Response &res)
{
  holding_pose = req.hold;
  if (holding_pose)
  {
    if (wam_dof == 4)
    {
      orientationSetPoint.setValue(wam4->getToolOrientation());
      wam4->trackReferenceSignal(orientationSetPoint.output);
    }
    else if (wam_dof == 7)
    {
      orientationSetPoint.setValue(wam7->getToolOrientation());
      wam7->trackReferenceSignal(orientationSetPoint.output);
    }
  }
  else
  {
    if (wam_dof == 4)
    {
      wam4->idle();
    }
    else if (wam_dof == 7)
    {
      wam7->idle();
    }
  }ROS_INFO("Pose Hold request: %s", (req.hold) ? "true" : "false");
  return true;
}

bool hold_joint_position(wam_msgs::HoldJointPosition::Request &req, wam_msgs::HoldJointPosition::Response &res)
{
  holding_j_pos = req.hold;
  if (holding_j_pos)
  {
    if (wam_dof == 4){
      wam4->moveTo(wam4->getJointPositions());
    }
    else if (wam_dof == 7)
    {
      wam7->moveTo(wam7->getJointPositions());
    }
  }
  else
  {
    if (wam_dof == 4)
    {
      wam4->idle();
    }
    else if (wam_dof == 7)
    {
      wam7->idle();
    }
  }ROS_INFO("Joint Position Hold request: %s", (req.hold) ? "true" : "false");
  return true;
}

bool WamNode::gravity(wam_msgs::GravityComp::Request &req, wam_msgs::GravityComp::Response &res)
{
  grav_state = req.gravity;
  if (wam4 != NULL)
    wam4->gravityCompensate(grav_state);
  else if (wam7 != NULL)
    wam7->gravityCompensate(grav_state);

  ROS_INFO("Gravity Compensation Request: %s", (req.gravity) ? "true" : "false");
  return true;
}

bool WamNode::joint_move(wam_msgs::JointMove::Request &req, wam_msgs::JointMove::Response &res)
{
  if (req.joints.size() != (size_t)wam_dof)
  {
    ROS_INFO("Request Failed: %d-DOF request received, must be %d-DOF", req.joints.size(), wam_dof);
    return false;
  }

  if (wam_dof == 4)
  {
    for (int i = 0; i < 4; i++)
      (*jp4c)[i] = req.joints[i];
    wam4->moveTo(*jp4c, false);
  }
  else if (wam_dof == 7)
  {
    for (int j = 0; j < 7; j++)
    {
      (*jp7c)[j] = req.joints[j];
    }
    wam7->moveTo(*jp7c, false);
  }ROS_INFO("Moving Robot to Commanded Joint Pose");
  return true;
}

bool WamNode::cart_move(wam_msgs::CartMove::Request &req, wam_msgs::CartMove::Response &res)
{
  if (req.coordinates.size() != 3)
  {
    ROS_INFO("Request Failed: %d coordinate request received, must be 3 coordinates [X,Y,Z]", req.coordinates.size());
    return false;
  }
  for (int i = 0; i < 3; i++)
    (*cpc)[i] = req.coordinates[i];

  if (wam_dof == 4)
    wam4->moveTo(*cpc, false);
  else
    wam7->moveTo(*cpc, false);

  ROS_INFO("Moving Robot to Commanded Coordinate Pose");
  return true;
}

bool WamNode::bhand_spread_pos(wam_msgs::BHandSpreadPos::Request &req, wam_msgs::BHandSpreadPos::Response &res)
{
  hand->spreadMove(Hand::jp_type(req.radians), false);
  ROS_INFO("Moving Barrett Hand to commanded spread position: %f", req.radians);
  return true;
}

bool WamNode::bhand_grasp_pos(wam_msgs::BHandGraspPos::Request &req, wam_msgs::BHandGraspPos::Response &res)
{
  hand->graspMove(Hand::jp_type(req.radians), false);
  ROS_INFO("Moving Barrett Hand to commanded grasp position: %f", req.radians);
  return true;
}

bool WamNode::bhand_spread_vel(wam_msgs::BHandSpreadVel::Request &req, wam_msgs::BHandSpreadVel::Response &res)
{
  hand->spreadVelocity(Hand::jv_type(req.velocity));
  ROS_INFO("Moving Barrett Hand commanded spread velocity: %f", req.velocity);
  return true;
}

bool WamNode::bhand_grasp_vel(wam_msgs::BHandGraspVel::Request &req, wam_msgs::BHandGraspVel::Response &res)
{
  hand->graspVelocity(Hand::jv_type(req.velocity));
  ROS_INFO("Moving Barrett Hand commanded grasp velocity: %f", req.velocity);
  return true;
}

template<size_t DOF>
  void WamNode::update_publisher(ProductManager& pm)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    systems::Wam < DOF > &wam = *getWam<DOF>(pm);

    //publishing sensor_msgs/JointState to wam_joint
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp = wam.getToolPosition();
    toc = wam.getToolOrientation();

    for (int i = 0; i < wam_dof; i++)
    {
      wam_joint_state.position[i] = jp[i];
      wam_joint_state.velocity[i] = jv[i];
      wam_joint_state.effort[i] = jt[i];
    }
    wam_joint_pub.publish(wam_joint_state);
    //publishing geometry_msgs/PoseStamed to wam_pose
    wam_pose.pose.position.x = cp[0];
    wam_pose.pose.position.y = cp[1];
    wam_pose.pose.position.z = cp[2];
    wam_pose.pose.orientation.w = toc.w();
    wam_pose.pose.orientation.x = toc.x();
    wam_pose.pose.orientation.y = toc.y();
    wam_pose.pose.orientation.z = toc.z();
    pose_pub.publish(wam_pose);

    //publishing hand data if found
    if (got_hand)
    {
      Hand::jp_type hjpi = hand->getInnerLinkPosition();
      Hand::jp_type hjpo = hand->getOuterLinkPosition();
      for (int j = 0; j < 4; j++)
        bhand_joint_state.position[j] = hjpi[j];
      for (int k = 0; k < 3; k++)
        bhand_joint_state.position[k + 4] = hjpo[k];
      bhand_joint_pub.publish(bhand_joint_state);
    }

    //This is for the RT cart control
    if (last_recieved_rt_pos_msg_time + rt_pos_msg_timeout > ros::Time::now())
    {
      rt_status = true;

      if (newpos)
      {
        positionSetPoint.setValue(cp_type(rt_pos_cmd[0], rt_pos_cmd[1], rt_pos_cmd[2]));
        wam.trackReferenceSignal(positionSetPoint.output);
      }
      newpos = false;
    }
    else
    {
      if (rt_status == true)
      {
        wam.idle();
      }
      rt_status = false;
    }
  }

template<size_t DOF>
  int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    wam.gravityCompensate(grav_state);
    ros::init(argc, argv, "barrettWam");
    WamNode wam_node;
    wam_node.init<DOF>(pm);
    ros::Rate pub_rate(PUBLISH_FREQ);
    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
    {
      ros::spinOnce();
      wam_node.update_publisher<DOF>(pm);
      pub_rate.sleep();
    }
    return 0;
  }

