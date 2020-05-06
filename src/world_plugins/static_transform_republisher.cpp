/**  \file
     \brief Defines class that republish tf messages from sensors using tf2_static_broadcaster.
     \author Vojtěch Spurný  - vojtech.spurny@fel.cvut.cz (original implementation)
 */

// Include the headers
#include <gazebo/common/Plugin.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/callback_queue.h>

namespace gazebo
{

class GazeboStaticTransformRepublisher : public WorldPlugin {
public:
  GazeboStaticTransformRepublisher();

  virtual ~GazeboStaticTransformRepublisher();

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

private:
  ros::NodeHandle                              nh_;
  ros::Subscriber                              tf_static_listener_;
  tf2_ros::StaticTransformBroadcaster *        static_broadcaster_;
  std::vector<geometry_msgs::TransformStamped> transform_message_;

  void callbackTFGazeboStatic(const tf2_msgs::TFMessageConstPtr &msg);
};

GazeboStaticTransformRepublisher::GazeboStaticTransformRepublisher() : nh_("static_transform_republisher") {
}

GazeboStaticTransformRepublisher::~GazeboStaticTransformRepublisher() {
  delete static_broadcaster_;
}

void GazeboStaticTransformRepublisher::Load([[maybe_unused]] physics::WorldPtr world, [[maybe_unused]] sdf::ElementPtr sdf) {
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  tf_static_listener_ =
      nh_.subscribe("/tf_gazebo_static", 10, &GazeboStaticTransformRepublisher::callbackTFGazeboStatic, this, ros::TransportHints().tcpNoDelay());
  static_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();

  ROS_INFO("[GazeboStaticTransformRepublisher]: Initialized");
}

void GazeboStaticTransformRepublisher::callbackTFGazeboStatic(const tf2_msgs::TFMessageConstPtr &msg) {
  for (auto &transform : transform_message_) {
    if (transform.child_frame_id == msg->transforms.at(0).child_frame_id) {
      /* ROS_INFO("contains"); */
      return;
    }
  }

  transform_message_.insert(transform_message_.end(), msg->transforms.begin(), msg->transforms.end());

  static_broadcaster_->sendTransform(transform_message_);
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboStaticTransformRepublisher)
}  // namespace gazebo
