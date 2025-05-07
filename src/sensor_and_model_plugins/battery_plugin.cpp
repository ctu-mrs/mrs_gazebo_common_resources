/**  \file
     \brief Defines class that applies linear drag force to the uav body.
     \inspiration https://www.theconstructsim.com/q-a-190-air-drag-in-gazebo/
     \author Ondrej Prochazka - prochon4@fel.cvut.cz
 */

#include "common.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <gazebo/common/common.hh>

#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/physics/physics.hh>

#include <gazebo/transport/transport.hh>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include "CommandMotorSpeed.pb.h"

#include <ignition/math/Vector3.hh>

#include <thread>

#include <functional>
#include <propulsion_module.hpp>
#include <sensor_msgs/BatteryState.h>

namespace gazebo
{

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
/* Class definition //{ */
class BatteryPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate();

  void PublishBatteryStats();

private:
  void QueueThread();

  physics::ModelPtr model;  // Pointer to the model

  event::ConnectionPtr updateConnection;  // Pointer to the update event connection

  std::unique_ptr<ros::NodeHandle> rosNode;  // node use for ROS transport

  ros::CallbackQueue rosQueue;  // ROS callbackqueue that helps process messages
  std::thread rosQueueThread;  // thread the keeps running the rosQueue

  physics::LinkPtr link_to_apply_resistance;  // ROS subscriber

  std::string BatteryPluginTopicName = "battery_plugin";


#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d now_lin_vel;
#else
  math::Vector3 now_lin_vel;
#endif

  double battery_voltage_ = 0.0;
  std::string propulsion_module_config_;
  bool   verbose    = false;
  float last_time = 0.0;
  physics::WorldPtr world;  // The parent World
  transport::SubscriberPtr command_sub_;
  ros::Publisher battery_state_pub_;
  transport::NodePtr node_handle_;  // node use for ROS transport
  std::string namespace_;           // namespace for the node
  std::string command_sub_topic_;
  std::string propulsion_config_file_;
  PropulsionModule propulsion_module_;
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
  sensor_msgs::BatteryState battery_state_msg_;
  std::vector<double> throttles;
  size_t NUM_MOTORS = 4;
};

//}

/* Load() //{ */
void BatteryPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);
  std::cout << "Using Battery Resistance PLugin" << std::endl;

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + _parent->GetName() + command_sub_topic_, &BatteryPlugin::VelocityCallback, this);

  /* load elements //{ */
  if (_sdf->HasElement("propulsion_config_file")) {
    propulsion_config_file_ = _sdf->Get<std::string>("propulsion_config_file");
  } else {
    ROS_WARN(
        "[BatteryPlugin]: No Propulsion Config File given, setting default "
        "name %s",
        propulsion_config_file_.c_str());
  }
  propulsion_module_.initialize(ros::package::getPath("controller_module") + std::string("/") + propulsion_config_file_);

  if (_sdf->HasElement("battery_voltage")) {
    battery_voltage_ = _sdf->Get<double>("battery_voltage");
  } else {
    ROS_WARN(
        "[BatteryPlugin]: No battery_voltage given, setting default "
        "name %f",
        battery_voltage_);
  }

    propulsion_module_.initBattery(battery_voltage_, ros::Time::now().toSec());

  if (_sdf->HasElement("verbose")) {
    verbose = _sdf->Get<double>("verbose");
  } else {
    ROS_WARN(
        "[BatteryPlugin]: No verbose Given, setting default "
        "name %d",
        verbose);
  }

  if (verbose) {
    ROS_INFO("[BatteryPlugin]: All elements loaded successfully!");
  }

  //}

  // Store the pointer to the model
  model                    = _parent;
  world                    = model->GetWorld();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BatteryPlugin::OnUpdate, this));

  // Create a topic name
  // std::string fluid_resistance_index_topicName = "/fluid_resistance_index";

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized()) {
    int    argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "fluid_resistance_node", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  rosNode.reset(new ros::NodeHandle("fluid_resistance_node"));
  battery_state_pub_ = rosNode->advertise<sensor_msgs::BatteryState>("gazebo_battery_state", 1);

#if (GAZEBO_MAJOR_VERSION >= 8)
  last_time = world->SimTime().Float();
#else
  last_time          = world->GetSimTime().Float();
#endif


  // Spin up the queue helper thread.
  rosQueueThread = std::thread(std::bind(&BatteryPlugin::QueueThread, this));

}
//}

/* OnUpdate() //{ */
// Called by the world update start event
void BatteryPlugin::OnUpdate() {
    // The previous method has an issue that the update rate is set by xacro file and the physics engine update rate is different.
    // Every time the physics engine computes its forces, it needs to be given these forces on every update cycle. A force that
    // was applied on the previous update cycle is not automatically applied on the next update cycle. This is why the force needs
    // to be applied on every update cycle. The update cycle can be synced with the physics engine by using the OnUpdate method.
    // Found by Parakh and detailed at https://answers.ros.org/question/12521/applying-wrench-to-object-and-update-rate-of-gazebo_ros_force-plugin/
    PublishBatteryStats();
}
//}


/* PublishBatteryStats() //{ */
void BatteryPlugin::PublishBatteryStats() {

    double current = propulsion_module_.getTotalCurrentFromThrottles(throttles);
    propulsion_module_.updateBatterySoc(current, ros::Time::now().toSec());
    battery_state_msg_.header.stamp = ros::Time::now();
    battery_state_msg_.voltage = propulsion_module_.getVccFromThrottles(throttles);
    battery_state_msg_.current = propulsion_module_.getTotalCurrentFromThrottles(throttles);
    battery_state_msg_.capacity = propulsion_module_.getBatterySoc();
    battery_state_pub_.publish(battery_state_msg_);
    //std::cout << "Publishing Battery State" << std::endl;

}

/* VelocityCallback() //{  */
void BatteryPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  std::cout << rot_velocities->motor_speed_size() << std::endl;
  for (int i; i < NUM_MOTORS; i++) {
    std::cout << rot_velocities->motor_speed(i) << " ";
  }
  throttles.resize(NUM_MOTORS);
  for (int i = 0; i < NUM_MOTORS; i++) {
    throttles.at(i) = rot_velocities->motor_speed(i);
  }
}
//}
//}

/* QueueThread() //{ */
/// \brief ROS helper function that processes messages
void BatteryPlugin::QueueThread() {
  static const double timeout = 0.01;
  while (rosNode->ok()) {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
//}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin)

}  // namespace gazebo
