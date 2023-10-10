// Include the headers
#include <gazebo/common/Plugin.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "common.h"
#include "Float.pb.h"

#include <ros/ros.h>

#include <std_msgs/Float64.h>

namespace gazebo
{

typedef const boost::shared_ptr<const std_msgs::msgs::Float> MotorSpeedPtr;

class MotorSpeedRepublisher : public ModelPlugin {
public:
  MotorSpeedRepublisher();
  ~MotorSpeedRepublisher();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  std::string namespace_;
  std::string model_name_;
  std::string motor_speed_topic_;

  transport::NodePtr       node_handle_;
  transport::SubscriberPtr motor_speed_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;

  boost::thread callback_queue_thread_;
  void          callbackMotorSpeed(MotorSpeedPtr &msg_in);

  ros::NodeHandle ros_nh_;
  ros::Publisher  motor_speed_pub_;
};

MotorSpeedRepublisher::MotorSpeedRepublisher() {
}

MotorSpeedRepublisher::~MotorSpeedRepublisher() {
}

void MotorSpeedRepublisher::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  model_name_ = model_->GetName();

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else {
    gzerr << "[motor_speed_republisher] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("motorSpeedTopic")) {
    getSdfParam<std::string>(_sdf, "motorSpeedTopic", motor_speed_topic_, motor_speed_topic_);
  } else {
    gzerr << "[motor_speed_republisher] Please specify motorSpeedTopic.\n";
  }

  motor_speed_sub_ = node_handle_->Subscribe<std_msgs::msgs::Float>("~/" + model_name_ + motor_speed_topic_, &MotorSpeedRepublisher::callbackMotorSpeed, this);

  ros_nh_        = ros::NodeHandle("~");
  motor_speed_pub_ = ros_nh_.advertise<std_msgs::Float64>("/" + model_name_ + "/" + motor_speed_topic_, 1);

  gzmsg << "[motor_speed_republisher_plugin]: Initialized.\n";
}

/*//{ callbackMotorSpeed() */
void MotorSpeedRepublisher::callbackMotorSpeed(MotorSpeedPtr &msg_in) {

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif

  std_msgs::Float64 msg_out;
  msg_out.data = msg_in->data();
  try {
    motor_speed_pub_.publish(msg_out);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", motor_speed_pub_.getTopic().c_str());
  }
}
/*//}*/

GZ_REGISTER_MODEL_PLUGIN(MotorSpeedRepublisher);
}  // namespace gazebo
