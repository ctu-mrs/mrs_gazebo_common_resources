/**  \file
     \brief Defines class that applies linear drag force to the uav body.
     \inspiration https://www.theconstructsim.com/q-a-190-air-drag-in-gazebo/
     \author Ondrej Prochazka - prochon4@fel.cvut.cz
 */

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

#include <ignition/math/Vector3.hh>

#include <thread>

#include <functional>

namespace gazebo
{

/* Class definition //{ */
class FluidResistancePlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate();
  void UpdateLinearVel();
  void OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg);

  void ApplyResistance();

private:
  void QueueThread();

private:
  physics::ModelPtr model;  // Pointer to the model

private:
  event::ConnectionPtr updateConnection;  // Pointer to the update event connection

  // TODO: remove this variable and exchange it for x_res, ... to be abel update by topic
  double fluid_resistance_index = 1.0;

private:
  std::unique_ptr<ros::NodeHandle> rosNode;  // node use for ROS transport

private:
  ros::Subscriber rosSub;  // ROS subscriber
private:
  ros::CallbackQueue rosQueue;  // ROS callbackqueue that helps process messages
private:
  std::thread rosQueueThread;  // thread the keeps running the rosQueue

private:
  physics::LinkPtr link_to_apply_resistance;  // ROS subscriber

private:
  std::string fluid_resistanceTopicName = "fluid_resistance";

private:
  std::string NameLinkToApplyResistance = "base_link";

private:
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d now_lin_vel;
#else
  math::Vector3 now_lin_vel;
#endif

private:
  double rate  = 1.0;
  double res_x = 1.0;
  double res_y = 1.0;
  double res_z = 1.0;

private:
  float last_time = 0.0;

private:
  physics::WorldPtr world;  // The parent World
};

//}

/* Load() //{ */
void FluidResistancePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  std::cout << "Using Fluid Resistance PLugin" << std::endl;

  /* load elements //{ */
  if (_sdf->HasElement("fluid_resistanceTopicName")) {
    fluid_resistanceTopicName = _sdf->Get<std::string>("fluid_resistanceTopicName");
  } else {
    ROS_WARN("No Topic Given name given, setting default name %s", fluid_resistanceTopicName.c_str());
  }

  if (_sdf->HasElement("NameLinkToApplyResistance")) {
    NameLinkToApplyResistance = _sdf->Get<std::string>("NameLinkToApplyResistance");
  } else {
    ROS_WARN(
        "No NameLinkToApplyResistance Given name given, setting default "
        "name %s",
        NameLinkToApplyResistance.c_str());
  }

  if (_sdf->HasElement("rate")) {
    rate = _sdf->Get<double>("rate");
  } else {
    ROS_WARN(
        "No rate Given name given, setting default "
        "name %f",
        rate);
  }

  if (_sdf->HasElement("res_x")) {
    res_x = _sdf->Get<double>("res_x");
  } else {
    ROS_WARN(
        "No res_x Given name given, setting default "
        "name %f",
        res_x);
  }

  if (_sdf->HasElement("res_y")) {
    res_y = _sdf->Get<double>("res_y");
  } else {
    ROS_WARN(
        "No res_y Given name given, setting default "
        "name %f",
        res_y);
  }

  if (_sdf->HasElement("res_z")) {
    res_z = _sdf->Get<double>("res_z");
  } else {
    ROS_WARN(
        "No res_z Given name given, setting default "
        "name %f",
        res_z);
  }

  ROS_WARN("All elements loaded successfully!");

  //}

  // Store the pointer to the model
  model                    = _parent;
  world                    = model->GetWorld();
  link_to_apply_resistance = model->GetLink(NameLinkToApplyResistance);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&FluidResistancePlugin::OnUpdate, this));

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

#if (GAZEBO_MAJOR_VERSION >= 8)
  last_time = world->SimTime().Float();
#else
  last_time          = world->GetSimTime().Float();
#endif

  // Freq
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
      fluid_resistanceTopicName, 1, boost::bind(&FluidResistancePlugin::OnRosMsg, this, _1), ros::VoidPtr(), &rosQueue);
  rosSub = rosNode->subscribe(so);

  // Spin up the queue helper thread.
  rosQueueThread = std::thread(std::bind(&FluidResistancePlugin::QueueThread, this));

  ROS_WARN(
      "Loaded FluidResistance Plugin with parent...%s, With Fluid "
      "Resistance = %f "
      "Started ",
      model->GetName().c_str(), fluid_resistance_index);
}
//}

/* OnUpdate() //{ */
// Called by the world update start event
void FluidResistancePlugin::OnUpdate() {
  float period = 1.0 / rate;

  // Get simulator time
#if (GAZEBO_MAJOR_VERSION >= 8)
  float current_time = world->SimTime().Float();
#else
  float current_time = world->GetSimTime().Float();
#endif
  float dt = current_time - last_time;

  if (dt <= period) {
    ROS_DEBUG(">>>>>>>>>>TimePassed = %f, TimePeriod =%f ", dt, period);
    return;
  } else {
    last_time = current_time;
    ApplyResistance();
  }
}
//}

/* UpdateLinearVel() //{ */
void FluidResistancePlugin::UpdateLinearVel() {
#if (GAZEBO_MAJOR_VERSION >= 8)
  now_lin_vel = model->RelativeLinearVel();
#else
  now_lin_vel        = model->GetRelativeLinearVel();
#endif
}
//}

/* ApplyResistance() //{ */
void FluidResistancePlugin::ApplyResistance() {

  UpdateLinearVel();

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d force, torque;
#else
  math::Vector3 force, torque;
#endif

  ROS_WARN("LinearSpeed = [%f,%f,%f] ", now_lin_vel.X(), now_lin_vel.Y(), now_lin_vel.Z());

  force.X(-1.0 * res_x * now_lin_vel.X());
  force.Y(-1.0 * res_y * now_lin_vel.Y());
  force.Z(-1.0 * res_z * now_lin_vel.Z());

  // Applying force to uav
  link_to_apply_resistance->AddRelativeForce(force);
#if (GAZEBO_MAJOR_VERSION >= 8)
  link_to_apply_resistance->AddRelativeTorque(torque - link_to_apply_resistance->GetInertial()->CoG().Cross(force));
#else
  link_to_apply_resistance->AddRelativeTorque(torque - link_to_apply_resistance->GetInertial()->GetCoG().Cross(force));
#endif

  ROS_WARN("FluidResistanceApplying = [%f,%f,%f] ", force.X(), force.Y(), force.Z());
}
//}

/* OnRosMsg() //{ */
void FluidResistancePlugin::OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg) {
  /* UpdateResistanceValues(_msg->data); */
  res_x = _msg->x;
  res_y = _msg->y;
  res_z = _msg->z;
  ROS_WARN("fluid resistance changed!!");
}
//}

/* QueueThread() //{ */
/// \brief ROS helper function that processes messages
void FluidResistancePlugin::QueueThread() {
  static const double timeout = 0.01;
  while (rosNode->ok()) {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
//}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(FluidResistancePlugin)

}  // namespace gazebo
