#include <unistd.h>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "wam_msgs/RTJointPos.h"
#include "wam_msgs/RTJointVel.h"
#include "wam_msgs/RTCartPos.h"
#include "wam_msgs/RTCartVel.h"
#include "wam_msgs/RTOrtnPos.h"
#include "wam_msgs/RTOrtnVel.h"
#include "wam_srvs/GravityComp.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/JointMove.h"
#include "wam_srvs/PoseMove.h"
#include "wam_srvs/CartPosMove.h"
#include "wam_srvs/OrtnMove.h"
#include "wam_srvs/BHandFingerPos.h"
#include "wam_srvs/BHandGraspPos.h"
#include "wam_srvs/BHandSpreadPos.h"
#include "wam_srvs/BHandFingerVel.h"
#include "wam_srvs/BHandGraspVel.h"
#include "wam_srvs/BHandSpreadVel.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "bypass_safety_module.h"

#include <barrett/math.h> 
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#define BARRETT_SMF_CONFIGURE_PM
#define BARRETT_SMF_DONT_PROMPT_ON_ZEROING
#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>

static const int PUBLISH_FREQ = 100; // Default Control Loop / Publishing Frequency
static const double SPEED = 0.03; // Default Cartesian Velocity

using namespace barrett;

//Some templated functions for 4-dof & 7-dof interchangeability
template<size_t DOF>
  systems::Wam<DOF>*
  getWam(ProductManager& pm)
  {
    assert(false);
    return NULL;
  }
template<>
  systems::Wam<7>*
  getWam(ProductManager& pm)
  {
    return pm.getWam7(false);
  }
template<>
  systems::Wam<4>*
  getWam(ProductManager& pm)
  {
    return pm.getWam4(false);
  }

//Creating a templated multiplier for our real-time computation
template<typename T1, typename T2, typename OutputType>
  class Multiplier : public systems::System, public systems::SingleOutput<OutputType>
  {
  public:
    Input<T1> input1;
  public:
    Input<T2> input2;

  public:
    Multiplier(std::string sysName = "Multiplier") :
        systems::System(sysName), systems::SingleOutput<OutputType>(this), input1(this), input2(this)
    {
    }
    virtual ~Multiplier()
    {
      mandatoryCleanUp();
    }

  protected:
    OutputType data;
    virtual void operate()
    {
      data = input1.getValue() * input2.getValue();
      this->outputValue->setData(&data);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Multiplier);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

//Creating a templated converter from Roll, Pitch, Yaw to Quaternion for real-time computation
class ToQuaternion : public systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>
{
public:
  Eigen::Quaterniond outputQuat;

public:
  ToQuaternion(std::string sysName = "ToQuaternion") :
      systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>(sysName)
  {
  }
  virtual ~ToQuaternion()
  {
    mandatoryCleanUp();
  }

protected:
  btQuaternion q;
  virtual void operate()
  {
    const math::Vector<3>::type &inputRPY = input.getValue();
    q.setEulerZYX(inputRPY[2], inputRPY[1], inputRPY[0]);
    outputQuat.x() = q.getX();
    outputQuat.y() = q.getY();
    outputQuat.z() = q.getZ();
    outputQuat.w() = q.getW();
    this->outputValue->setData(&outputQuat);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(ToQuaternion);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//Simple Function for converting Quaternion to RPY
math::Vector<3>::type toRPY(Eigen::Quaterniond inquat)
{
  math::Vector<3>::type newRPY;
  btQuaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
  btMatrix3x3(q).getEulerZYX(newRPY[2], newRPY[1], newRPY[0]);
  return newRPY;
}

//WamNode Class
class WamNode
{
protected:
  size_t wam_dof;
  bool cart_vel_status, ortn_vel_status, new_rt_cmd;
  double cart_vel_mag, ortn_vel_mag;
  double rt_cart_vel[3], rt_ortn_vel[3];
  systems::Wam<4>* wam4;
  systems::Wam<7>* wam7;
  Hand* hand;
  units::JointPositions<4>::type jp4, jp4_cmd, jp4_home;
  units::JointPositions<7>::type jp7, jp7_cmd, jp7_home;
  units::CartesianPosition::type cp_cmd;
  Eigen::Quaterniond ortn_cmd;
  boost::tuple<units::CartesianPosition::type, Eigen::Quaterniond> pose_cmd;
  systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint, current_ortn;
  systems::ExposedOutput<units::CartesianPosition::type> cart_dir, current_cart_pos;
  systems::ExposedOutput<math::Vector<3>::type> rpy_cmd, current_rpy_ortn;
  systems::TupleGrouper<units::CartesianPosition::type, Eigen::Quaterniond> rt_pose_cmd;
  systems::Summer<units::CartesianPosition::type> cart_pos_sum;
  systems::Summer<math::Vector<3>::type> ortn_cmd_sum;
  systems::Ramp ramp;
  Multiplier<double, units::CartesianPosition::type, units::CartesianPosition::type> mult_linear;
  Multiplier<double, math::Vector<3>::type, math::Vector<3>::type> mult_angular;
  ToQuaternion to_quat, to_quat_print;
  Eigen::Quaterniond ortn_print;
  ros::Time last_cart_vel_msg_time, last_ortn_vel_msg_time;
  ros::Duration rt_msg_timeout;

  //Subscribed Topics
  wam_msgs::RTCartVel cart_vel_cmd;
  wam_msgs::RTOrtnVel ortn_vel_cmd;

  //Subscribers
  ros::Subscriber cart_vel_sub;
  ros::Subscriber ortn_vel_sub;

  //Published Topics
  sensor_msgs::JointState wam_joint_state, bhand_joint_state;
  geometry_msgs::PoseStamped wam_pose;

  //Publishers
  ros::Publisher wam_joint_state_pub, bhand_joint_state_pub, wam_pose_pub;

  //Services
  ros::ServiceServer gravity_srv, go_home_srv, hold_jpos_srv, hold_cpos_srv;
  ros::ServiceServer hold_ortn_srv, joint_move_srv, pose_move_srv;
  ros::ServiceServer cart_move_srv, ortn_move_srv, hand_close_srv;
  ros::ServiceServer hand_open_grsp_srv, hand_close_grsp_srv, hand_open_sprd_srv;
  ros::ServiceServer hand_close_sprd_srv, hand_fngr_pos_srv, hand_fngr_vel_srv;
  ros::ServiceServer hand_grsp_pos_srv, hand_grsp_vel_srv, hand_sprd_pos_srv;
  ros::ServiceServer hand_sprd_vel_srv;

public:
  WamNode() :
      ramp(NULL, SPEED)
  {
  }

  template<size_t DOF>
    void
    init(ProductManager& pm);

  ~WamNode()
  {
  }

  bool
  gravity(wam_srvs::GravityComp::Request &req, wam_srvs::GravityComp::Response &res);
  bool
  goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool
  holdJPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
  bool
  holdCPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
  bool
  holdOrtn(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
  bool
  jointMove(wam_srvs::JointMove::Request &req, wam_srvs::JointMove::Response &res);
  bool
  poseMove(wam_srvs::PoseMove::Request &req, wam_srvs::PoseMove::Response &res);
  bool
  cartMove(wam_srvs::CartPosMove::Request &req, wam_srvs::CartPosMove::Response &res);
  bool
  ortnMove(wam_srvs::OrtnMove::Request &req, wam_srvs::OrtnMove::Response &res);
  bool
  handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool
  handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool
  handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool
  handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool
  handFingerPos(wam_srvs::BHandFingerPos::Request &req, wam_srvs::BHandFingerPos::Response &res);
  bool
  handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res);
  bool
  handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res);
  bool
  handFingerVel(wam_srvs::BHandFingerVel::Request &req, wam_srvs::BHandFingerVel::Response &res);
  bool
  handGraspVel(wam_srvs::BHandGraspVel::Request &req, wam_srvs::BHandGraspVel::Response &res);
  bool
  handSpreadVel(wam_srvs::BHandSpreadVel::Request &req, wam_srvs::BHandSpreadVel::Response &res);
  void
  cartVelCB(const wam_msgs::RTCartVel::ConstPtr& msg);
  void
  ortnVelCB(const wam_msgs::RTOrtnVel::ConstPtr& msg);
  template<size_t DOF>
    void
    publish(ProductManager& pm);
  template<size_t DOF>
    void
    updateRT(ProductManager& pm);
};

// Templated Initialization Function
template<size_t DOF>
  void WamNode::init(ProductManager& pm)
  { 
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    wam4 = NULL;
    wam7 = NULL;
    hand = NULL;

    systems::Wam<DOF> &wam = *getWam<DOF>(pm);

    ros::NodeHandle n_("wam"); // WAM specific nodehandle
    ros::NodeHandle nh_("bhand"); // BarrettHand specific nodehandle

    wam_dof = DOF; //global wam_dof to be used in functions that cannot be templated

    //Setting up real-time command timeouts and initial values
    cart_vel_status = false; //Bool for determining cartesian velocity real-time state
    ortn_vel_status = false; //Bool for determining orientation velocity real-time state
    new_rt_cmd = false; //Bool for determining if a new real-time message was received
    rt_msg_timeout.fromSec(0.3); //rt_status will be determined false if rt message is not received in specified time
    cart_vel_mag = SPEED; //Setting default cartesian velocity magnitude to SPEED
    ortn_vel_mag = SPEED;
    pm.getExecutionManager()->startManaging(ramp); //starting ramp manager

    if (DOF == 4) //4-DOF specific initialization
    {
      std::cout << "4-DOF WAM" << std::endl;
      wam4 = pm.getWam4(false); //pointer to systems::Wam<4> object for functions that cannot be templated 
      jp4_home = wam4->getJointPositions();
    }
    else if (DOF == 7) //7-DOF specific initialization
    {
      std::cout << "7-DOF WAM" << std::endl;
      wam7 = pm.getWam7(false); //pointer to systems::Wam<7> object for functions that cannot be templated 
      jp7_home = wam7->getJointPositions();
    }

    if (pm.foundHand()) //Does the following only if a BarrettHand is present
    {
      std::cout << "Barrett Hand" << std::endl;
      hand = pm.getHand();
      // Move j3 in order to give room for hand initialization
      jp_type jp_init = wam.getJointPositions();
      jp_init[3] -= 0.35;
      wam.moveTo(jp_init);

      hand->initialize();
      hand->update(Hand::S_ALL, true);
      hand->update();

      //Publishing the following topics only if there is a BarrettHand present
      bhand_joint_state_pub = nh_.advertise < sensor_msgs::JointState > ("joint_states", 1); // bhand/joint_states

      //Advertise the following services only if there is a BarrettHand present
      hand_open_grsp_srv = nh_.advertiseService("open_grasp", &WamNode::handOpenGrasp, this); // bhand/open_grasp
      hand_close_grsp_srv = nh_.advertiseService("close_grasp", &WamNode::handCloseGrasp, this); // bhand/close_grasp
      hand_open_sprd_srv = nh_.advertiseService("open_spread", &WamNode::handOpenSpread, this); // bhand/open_spread
      hand_close_sprd_srv = nh_.advertiseService("close_spread", &WamNode::handCloseSpread, this); // bhand/close_spread
      hand_fngr_pos_srv = nh_.advertiseService("finger_pos", &WamNode::handFingerPos, this); // bhand/finger_pos
      hand_grsp_pos_srv = nh_.advertiseService("grasp_pos", &WamNode::handGraspPos, this); // bhand/grasp_pos
      hand_sprd_pos_srv = nh_.advertiseService("spread_pos", &WamNode::handSpreadPos, this); // bhand/spread_pos
      hand_fngr_vel_srv = nh_.advertiseService("finger_vel", &WamNode::handFingerVel, this); // bhand/finger_vel
      hand_grsp_vel_srv = nh_.advertiseService("grasp_vel", &WamNode::handGraspVel, this); // bhand/grasp_vel
      hand_sprd_vel_srv = nh_.advertiseService("spread_vel", &WamNode::handSpreadVel, this); // bhand/spread_vel

      //Set up the BarrettHand joint state publisher
      const char* bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread", "outer_f1", "outer_f2", "outer_f3"};
      std::vector<std::string> bhand_joints(bhand_jnts, bhand_jnts + 7);
      bhand_joint_state.name.resize(7);
      bhand_joint_state.name = bhand_joints;
      bhand_joint_state.position.resize(7);
    }

    //Setting up WAM joint state publisher
    const char* wam_jnts[] = {"wam_j1", "wam_j2", "wam_j3", "wam_j4", "wam_j5", "wam_j6", "wam_j7"};
    std::vector<std::string> wam_joints(wam_jnts, wam_jnts + 7);
    wam_joint_state.name = wam_joints;
    wam_joint_state.name.resize(DOF);
    wam_joint_state.position.resize(DOF);
    wam_joint_state.velocity.resize(DOF);
    wam_joint_state.effort.resize(DOF);

    //Publishing the following rostopics
    wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1); // wam/joint_states
    wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1); // wam/pose    

    //Subscribing to the following rostopics
    cart_vel_sub = n_.subscribe("cart_vel_cmd", 1, &WamNode::cartVelCB, this); // wam/cart_vel_cmd
    ortn_vel_sub = n_.subscribe("ortn_vel_cmd", 1, &WamNode::ortnVelCB, this); // wam/ortn_vel_cmd

    //Advertising the following rosservices
    gravity_srv = n_.advertiseService("gravity_comp", &WamNode::gravity, this); // wam/gravity_comp
    go_home_srv = n_.advertiseService("go_home", &WamNode::goHome, this); // wam/go_home 
    hold_jpos_srv = n_.advertiseService("hold_joint_pos", &WamNode::holdJPos, this); // wam/hold_joint_pos
    hold_cpos_srv = n_.advertiseService("hold_cart_pos", &WamNode::holdCPos, this); // wam/hold_cart_pos
    hold_ortn_srv = n_.advertiseService("hold_ortn", &WamNode::holdOrtn, this); // wam/hold_ortn
    joint_move_srv = n_.advertiseService("joint_move", &WamNode::jointMove, this); // wam/joint_move
    pose_move_srv = n_.advertiseService("pose_move", &WamNode::poseMove, this); // wam/pose_move
    cart_move_srv = n_.advertiseService("cart_move", &WamNode::cartMove, this); // wam/cart_pos_move
    ortn_move_srv = n_.advertiseService("ortn_move", &WamNode::ortnMove, this); // wam/ortn_move
    
    std::cout << "Wam Node Active" << std::endl;

  }

// gravity_comp service callback
bool WamNode::gravity(wam_srvs::GravityComp::Request &req, wam_srvs::GravityComp::Response &res)
{
  if (wam4 != NULL)
    wam4->gravityCompensate(req.gravity);
  else if (wam7 != NULL)
    wam7->gravityCompensate(req.gravity);

  ROS_INFO("Gravity Compensation Request: %s", (req.gravity) ? "true" : "false");
  return true;
}

// goHome Function for sending the WAM safely back to its home starting position.
bool WamNode::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Returning to Home Position");

  if (hand != NULL)
  {
    hand->open(Hand::GRASP, true);
    hand->close(Hand::SPREAD, true);
  }
  if (wam_dof == 4)
  {
    for (int i = 0; i < 4; i++)
      jp4_cmd[i] = 0.0;
    wam4->moveTo(jp4_cmd, true);
    jp4_home[3] += 0.3;
    wam4->moveTo(jp4_home, true);
    jp4_home[3] -= 0.3;
    wam4->moveTo(jp4_home, true);
  }
  else if (wam_dof == 7)
  {
    for (int j = 0; j < 7; j++)
      jp7_cmd[j] = 0.0;
    wam7->moveTo(jp7_cmd, true);
    jp7_home[3] -= 0.3;
    wam7->moveTo(jp7_home, true);
    jp7_home[3] += 0.3;
    wam7->moveTo(jp7_home, true);
  }
  return true;
}

//Function to hold WAM Joint Positions
bool WamNode::holdJPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
  ROS_INFO("Joint Position Hold request: %s", (req.hold) ? "true" : "false");

  if (req.hold && wam_dof == 4)
    wam4->moveTo(wam4->getJointPositions());
  else if (req.hold && wam_dof == 7)
    wam7->moveTo(wam7->getJointPositions());
  else if (wam_dof == 4)
    wam4->idle();
  else if (wam_dof == 7)
    wam7->idle();
  return true;
}

//Function to hold WAM end effector Cartesian Position
bool WamNode::holdCPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
  ROS_INFO("Cartesian Position Hold request: %s", (req.hold) ? "true" : "false");

  if (req.hold && wam_dof == 4)
    wam4->moveTo(wam4->getToolPosition());
  else if (req.hold && wam_dof == 7)
    wam7->moveTo(wam7->getToolPosition());
  else if (wam_dof == 4)
    wam4->idle();
  else if (wam_dof == 7)
    wam7->idle();
  return true;
}

//Function to hold WAM end effector Orientation
bool WamNode::holdOrtn(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
  ROS_INFO("Orientation Hold request: %s", (req.hold) ? "true" : "false");

  if (req.hold && wam_dof == 4)
  {
    orientationSetPoint.setValue(wam4->getToolOrientation());
    wam4->trackReferenceSignal(orientationSetPoint.output);
  }
  else if (req.hold && wam_dof == 7)
  {
    orientationSetPoint.setValue(wam7->getToolOrientation());
    wam7->trackReferenceSignal(orientationSetPoint.output);
  }
  else if (wam_dof == 4)
    wam4->idle();
  else if (wam_dof == 7)
    wam7->idle();
  return true;
}

//Function to command a joint space move to the WAM
bool WamNode::jointMove(wam_srvs::JointMove::Request &req, wam_srvs::JointMove::Response &res)
{
  if (req.joints.size() != wam_dof)
  {
    ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), wam_dof);
    return false;
  }
  ROS_INFO("Moving Robot to Commanded Joint Pose");
  if (wam_dof == 4)
  {
    for (int i = 0; i < 4; i++)
      jp4_cmd[i] = req.joints[i];
    wam4->moveTo(jp4_cmd, false);
  }
  else if (wam_dof == 7)
  {
    for (int j = 0; j < 7; j++)
      jp7_cmd[j] = req.joints[j];
    wam7->moveTo(jp7_cmd, false);
  }
  return true;
}

//Function to command a pose move to the WAM
bool WamNode::poseMove(wam_srvs::PoseMove::Request &req, wam_srvs::PoseMove::Response &res)
{
  ROS_INFO("Moving Robot to Commanded Pose");

  cp_cmd[0] = req.pose.position.x;
  cp_cmd[1] = req.pose.position.y;
  cp_cmd[2] = req.pose.position.z;
  ortn_cmd.x() = req.pose.orientation.x;
  ortn_cmd.y() = req.pose.orientation.y;
  ortn_cmd.z() = req.pose.orientation.z;
  ortn_cmd.w() = req.pose.orientation.w;

  pose_cmd = boost::make_tuple(cp_cmd, ortn_cmd);

  if (wam_dof == 4)
  {
    //wam4->moveTo(pose_cmd, false); //(TODO:KM Update Libbarrett API for Pose Moves)
  }
  else if (wam_dof == 7)
  {
    //wam7->moveTo(pose_cmd, false);
  }
  return true;
}

//Function to command a cartesian move to the WAM
bool WamNode::cartMove(wam_srvs::CartPosMove::Request &req, wam_srvs::CartPosMove::Response &res)
{
  ROS_INFO("Moving Robot to Commanded Cartesian Position");

  for (int i = 0; i < 3; i++)
    cp_cmd[i] = req.position[i];
  if (wam_dof == 4)
  {
    wam4->moveTo(cp_cmd, false);
  }
  else if (wam_dof == 7)
  {
    wam7->moveTo(cp_cmd, false);
  }
  return true;
}

//Function to command an orientation move to the WAM
bool WamNode::ortnMove(wam_srvs::OrtnMove::Request &req, wam_srvs::OrtnMove::Response &res)
{
  ROS_INFO("Moving Robot to Commanded End Effector Orientation");

  ortn_cmd.x() = req.orientation[0];
  ortn_cmd.y() = req.orientation[1];
  ortn_cmd.z() = req.orientation[2];
  ortn_cmd.w() = req.orientation[3];

  if (wam_dof == 4)
  {
    wam4->moveTo(ortn_cmd, false);
  }
  else if (wam_dof == 7)
  {
    wam7->moveTo(ortn_cmd, false);
  }
  return true;
}

//Function to open the BarrettHand Grasp
bool WamNode::handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Opening the BarrettHand Grasp");
  hand->open(Hand::GRASP, false);
  return true;
}

//Function to close the BarrettHand Grasp
bool WamNode::handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Closing the BarrettHand Grasp");
  hand->close(Hand::GRASP, false);
  return true;
}

//Function to open the BarrettHand Spread
bool WamNode::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Opening the BarrettHand Spread");
  hand->open(Hand::SPREAD, false);
  return true;
}

//Function to close the BarrettHand Spread
bool WamNode::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Closing the BarrettHand Spread");
  hand->close(Hand::SPREAD, false);
  return true;
}

//Function to control a BarrettHand Finger Position
bool WamNode::handFingerPos(wam_srvs::BHandFingerPos::Request &req, wam_srvs::BHandFingerPos::Response &res)
{
  ROS_INFO("Moving BarrettHand Finger %d: %.3f radians", req.finger, req.radians);
  hand->trapezoidalMove(Hand::jp_type(req.radians), req.finger, false);
  return true;
}

//Function to control the BarrettHand Grasp Position
bool WamNode::handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res)
{
  ROS_INFO("Moving BarrettHand Grasp: %.3f radians", req.radians);
  hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::GRASP, false);
  return true;
}

//Function to control the BarrettHand Spread Position
bool WamNode::handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res)
{
  ROS_INFO("Moving BarrettHand Spread: %.3f radians", req.radians);
  hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::SPREAD, false);
  return true;
}

//Function to control a BarrettHand Finger Velocity
bool WamNode::handFingerVel(wam_srvs::BHandFingerVel::Request &req, wam_srvs::BHandFingerVel::Response &res)
{
  ROS_INFO("Moving BarrettHand Finger %d: %.3f m/s", req.finger, req.velocity);
  hand->velocityMove(Hand::jv_type(req.velocity), req.finger);
  return true;
}

//Function to control a BarrettHand Grasp Velocity
bool WamNode::handGraspVel(wam_srvs::BHandGraspVel::Request &req, wam_srvs::BHandGraspVel::Response &res)
{
  ROS_INFO("Moving BarrettHand Grasp: %.3f m/s", req.velocity);
  hand->velocityMove(Hand::jv_type(req.velocity), Hand::GRASP);
  return true;
}

//Function to control a BarrettHand Spread Velocity
bool WamNode::handSpreadVel(wam_srvs::BHandSpreadVel::Request &req, wam_srvs::BHandSpreadVel::Response &res)
{
  ROS_INFO("Moving BarrettHand Spread: %.3f m/s", req.velocity);
  hand->velocityMove(Hand::jv_type(req.velocity), Hand::SPREAD);
  return true;
}

//Callback function for RT Cartesian Velocity messages
void WamNode::cartVelCB(const wam_msgs::RTCartVel::ConstPtr& msg)
{
  if (cart_vel_status)
  {
    for (int i = 0; i < 3; i++)
      rt_cart_vel[i] = msg->direction[i];
    new_rt_cmd = true;
  }
  if (msg->magnitude != 0)
    cart_vel_mag = msg->magnitude;
  last_cart_vel_msg_time = ros::Time::now();
}

//Callback function for RT Orientation Velocity messages
void WamNode::ortnVelCB(const wam_msgs::RTOrtnVel::ConstPtr& msg)
{
  if (ortn_vel_status)
  {
    for (int i = 0; i < 3; i++)
      rt_ortn_vel[i] = msg->angular[i];
    new_rt_cmd = true;
  }
  if (msg->magnitude != 0)
    ortn_vel_mag = msg->magnitude;
  last_ortn_vel_msg_time = ros::Time::now();
}

//Function to update the publisher
template<size_t DOF>
  void WamNode::publish(ProductManager& pm)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    systems::Wam<DOF> &wam = *getWam<DOF>(pm);

    //Current values to be published
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp_pub = wam.getToolPosition();
    Eigen::Quaterniond to_pub = wam.getToolOrientation();

    //publishing sensor_msgs/JointState to wam/joint_states
    for (size_t i = 0; i < DOF; i++)
    {
      wam_joint_state.position[i] = jp[i];
      wam_joint_state.velocity[i] = jv[i];
      wam_joint_state.effort[i] = jt[i];
    }
    wam_joint_state.header.stamp = ros::Time::now();
    wam_joint_state_pub.publish(wam_joint_state);

    //publishing geometry_msgs/PoseStamed to wam/pose
    wam_pose.header.stamp = ros::Time::now();
    wam_pose.pose.position.x = cp_pub[0];
    wam_pose.pose.position.y = cp_pub[1];
    wam_pose.pose.position.z = cp_pub[2];
    wam_pose.pose.orientation.w = to_pub.w();
    wam_pose.pose.orientation.x = to_pub.x();
    wam_pose.pose.orientation.y = to_pub.y();
    wam_pose.pose.orientation.z = to_pub.z();
    wam_pose_pub.publish(wam_pose);

    //publishing sensor_msgs/JointState to bhand/joint_states if present
    if (hand != NULL)
    {
      Hand::jp_type hi = hand->getInnerLinkPosition();
      Hand::jp_type ho = hand->getOuterLinkPosition();
      for (size_t i = 0; i < 4; i++)
        bhand_joint_state.position[i] = hi[i];
      for (size_t j = 0; j < 3; j++)
        bhand_joint_state.position[j + 4] = ho[j];
      bhand_joint_state.header.stamp = ros::Time::now();
      bhand_joint_state_pub.publish(bhand_joint_state);
    }
  }

//Function to update the real-time control loops
template<size_t DOF>
  void WamNode::updateRT(ProductManager& pm)//systems::PeriodicDataLogger<debug_tuple>& logger
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    systems::Wam<DOF> &wam = *getWam<DOF>(pm);


    //Real-Time Cartesian Velocity Control Portion
    if (last_cart_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a cartesian velocity message has been published and if it is within timeout
    {    
      if (!cart_vel_status)
      {
        cart_dir.setValue(cp_type(0.0, 0.0, 0.0)); // zeroing the cartesian direction
        current_cart_pos.setValue(wam.getToolPosition()); // Initializing the cartesian position 
        current_ortn.setValue(wam.getToolOrientation()); // Initializing the orientation 
        systems::forceConnect(ramp.output, mult_linear.input1); // connecting the ramp to multiplier
        systems::forceConnect(cart_dir.output, mult_linear.input2); // connecting the direction to the multiplier
        systems::forceConnect(mult_linear.output, cart_pos_sum.getInput(0)); // adding the output of the multiplier
        systems::forceConnect(current_cart_pos.output, cart_pos_sum.getInput(1)); // with the starting cartesian position offset
        systems::forceConnect(cart_pos_sum.output, rt_pose_cmd.getInput<0>()); // saving summed position as new commanded pose.position
        systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>()); // saving the original orientation to the pose.orientation
        ramp.setSlope(cart_vel_mag); // setting the slope to the commanded magnitude
        ramp.stop(); // ramp is stopped on startup
        ramp.setOutput(0.0); // ramp is re-zeroed on startup
        ramp.start(); // start the ramp
        wam.trackReferenceSignal(rt_pose_cmd.output); // command WAM to track the RT commanded (500 Hz) updated pose
      }
      else if (new_rt_cmd)
      {
        BARRETT_SCOPED_LOCK(pm.getMutex());
        //Forces us into real-time
        ramp.reset(); // reset the ramp to 0
        ramp.setSlope(cart_vel_mag);
        cart_dir.setValue(cp_type(rt_cart_vel[0], rt_cart_vel[1], rt_cart_vel[2])); // set our cartesian direction to subscribed command
        current_cart_pos.setValue(wam.tpoTpController.referenceInput.getValue()); // updating the current position to the actual low level commanded value
      }
      cart_vel_status = true;
      new_rt_cmd = false;
    }

 
    //Real-Time Angular Velocity Control Portion
    else if (last_ortn_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a orientation velocity message has been published and if it is within timeout
    {
      if (!ortn_vel_status)
      {
        rpy_cmd.setValue(math::Vector<3>::type(0.0, 0.0, 0.0)); // zeroing the rpy command
        current_cart_pos.setValue(wam.getToolPosition()); // Initializing the cartesian position 
        current_rpy_ortn.setValue(toRPY(wam.getToolOrientation())); // Initializing the orientation 

        systems::forceConnect(ramp.output, mult_angular.input1); // connecting the ramp to multiplier
        systems::forceConnect(rpy_cmd.output, mult_angular.input2); // connecting the rpy command to the multiplier
        systems::forceConnect(mult_angular.output, ortn_cmd_sum.getInput(0)); // adding the output of the multiplier
        systems::forceConnect(current_rpy_ortn.output, ortn_cmd_sum.getInput(1)); // with the starting rpy orientation offset
        systems::forceConnect(ortn_cmd_sum.output, to_quat.input);
        systems::forceConnect(current_cart_pos.output, rt_pose_cmd.getInput<0>()); // saving the original position to the pose.position
        systems::forceConnect(to_quat.output, rt_pose_cmd.getInput<1>()); // saving the summed and converted new quaternion commmand as the pose.orientation
        ramp.setSlope(ortn_vel_mag); // setting the slope to the commanded magnitude
        ramp.stop(); // ramp is stopped on startup
        ramp.setOutput(0.0); // ramp is re-zeroed on startup
        ramp.start(); // start the ramp
        wam.trackReferenceSignal(rt_pose_cmd.output); // command the WAM to track the RT commanded (500 Hz) updated pose
      }
      else if (new_rt_cmd)
      {
        BARRETT_SCOPED_LOCK(pm.getMutex());//Forces us into real-time
        ramp.reset(); // reset the ramp to 0
        ramp.setSlope(ortn_vel_mag); // updating the commanded angular velocity magnitude
        rpy_cmd.setValue(math::Vector<3>::type(rt_ortn_vel[0], rt_ortn_vel[1], rt_ortn_vel[2])); // set our angular rpy command to subscribed command 
        current_rpy_ortn.setValue(toRPY(wam.tpoToController.referenceInput.getValue())); // updating the current orientation to the actual low level commanded value
        //current_rpy_ortn.setValue(toRPY(wam.getToolOrientation()));
      }
      ortn_vel_status = true;
      new_rt_cmd = false;
    }
    else if(cart_vel_status | ortn_vel_status){
        wam.moveTo(wam.getJointPositions()); // Holds current joint positions upon a RT message timeout 
        cart_vel_status = false;
        ortn_vel_status = false;
    }
  }

// Product Manager configuration necessary to preempt the standard main function, allowing for pendantless operation.
bool configure_pm(int argc, char** argv, ProductManager& pm)
{
   std::cout << "Starting WAM Node" << std::endl;

   SafetyModule* sm = pm.getSafetyModule();

   bypassSafetyModule(sm);

   SafetyModule::PendantState ps;
   
   sm->getPendantState(&ps);
   
   if(ps.pressedButton == SafetyModule::PendantState::ESTOP)
     std::cout << "Please Release the WAM ESTOP to start the wam_node" << std::endl;
   while(ps.pressedButton == SafetyModule::PendantState::ESTOP)
   {
     sm->getPendantState(&ps);
     usleep(100000);
   }
  
   sm->setMode(SafetyModule::IDLE);

   sm->waitForMode(SafetyModule::IDLE,false);

   return true;
}

//wam_main Function
template<size_t DOF>
  int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    
    usleep(250000);
    pm.getSafetyModule()->setMode(SafetyModule::ACTIVE);
    usleep(250000);

    wam.gravityCompensate(true); // Turning on Gravity Compenstation by Default when starting the WAM Node
    ros::init(argc, argv, "wam_node");
    WamNode wam_node;
    wam_node.init<DOF>(pm);
    ros::Rate pub_rate(PUBLISH_FREQ);

    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
    {
      ros::spinOnce();
      wam_node.publish<DOF>(pm); 
      wam_node.updateRT<DOF>(pm);
      pub_rate.sleep();
    }
    
    return 0;
  }

