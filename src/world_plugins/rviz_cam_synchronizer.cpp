/**  \file
     \brief Defines class that will synchronize rviz camera position and orientation according to gazebo camera.
     \author Petr Štibinger  - petr.stibinger@fel.cvut.cz (original implementation)
     \author Vojtěch Spurný  - vojtech.spurny@fel.cvut.cz (rewrite)
 */

// Include the headers
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/msgs/poses_stamped.pb.h>

namespace gazebo
{

typedef boost::shared_ptr<const msgs::Pose>         GazeboPoseConstPtr;
typedef boost::shared_ptr<const msgs::PosesStamped> GazeboPosesStampedConstPtr;

/* class CamSynchronizer //{ */

class CamSynchronizer : public WorldPlugin {
public:
  virtual ~CamSynchronizer();

  void Load(physics::WorldPtr model, sdf::ElementPtr sdf);

private:
  ignition::math::Vector3d target_position_;
  std::mutex               mutex_target_position_;
  std::atomic_bool         got_target_position_;

  // Gazebo subscribers
  transport::NodePtr       gz_node_;
  transport::SubscriberPtr gz_cam_sub_;
  transport::SubscriberPtr target_pose_sub_;

  // ROS publisher
  tf2_ros::TransformBroadcaster* broadcaster_;

  // parameters
  std::string _target_frame_id_;
  std::string _world_origin_frame_id_;
  std::string _frame_to_follow_;

  // callbacks
  void callbackGazeboCameraPose(const GazeboPoseConstPtr& msg);
  void callbackGazeboTargetPose(const GazeboPosesStampedConstPtr& msg);

  // help functions
  void                        publishTF(const ignition::math::Pose3d& cam_pose);
  ignition::math::Quaterniond getCamOrientation(const ignition::math::Vector3d& cam_position);
  ignition::math::Vector3d    msg2position(const GazeboPoseConstPtr& msg);
  geometry_msgs::Transform    pose2transform(const ignition::math::Pose3d& pose);
};

//}

/* ~CamSynchronizer() //{ */

CamSynchronizer::~CamSynchronizer() {
  delete broadcaster_;
}

//}

/* Load(...) //{ */

void CamSynchronizer::Load([[maybe_unused]] physics::WorldPtr model, sdf::ElementPtr sdf) {

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  _target_frame_id_       = sdf->Get<std::string>("target_frame_id");
  _world_origin_frame_id_ = sdf->Get<std::string>("world_origin_frame_id");
  _frame_to_follow_       = sdf->Get<std::string>("frame_to_follow");

  gz_node_ = transport::NodePtr(new transport::Node());
  gz_node_->Init();

  got_target_position_ = false;

  broadcaster_ = new tf2_ros::TransformBroadcaster();

  gz_cam_sub_      = gz_node_->Subscribe("~/user_camera/pose", &CamSynchronizer::callbackGazeboCameraPose, this, 1);
  target_pose_sub_ = gz_node_->Subscribe("~/pose/info", &CamSynchronizer::callbackGazeboTargetPose, this, 1);

  ROS_INFO("[GazeboRvizCamSynchronizer]: Initialized");
}

//}

/* callbackGazeboCameraPoseCallback(...) //{ */

void CamSynchronizer::callbackGazeboCameraPose(const GazeboPoseConstPtr& msg) {
  if (!got_target_position_.load()) {
    return;
  }

  const ignition::math::Vector3d    cam_position    = msg2position(msg);
  const ignition::math::Quaterniond cam_orientation = getCamOrientation(cam_position);

  const ignition::math::Pose3d cam_pose(cam_position, cam_orientation);
  publishTF(cam_pose);
}

//}

/* callbackGazeboTargetPose(...) //{ */

void CamSynchronizer::callbackGazeboTargetPose(const GazeboPosesStampedConstPtr& msg) {
  for (int i = 0; i < msg->pose_size(); i++) {
    if (msg->pose(i).name() == _frame_to_follow_) {
      std::scoped_lock lock(mutex_target_position_);
      target_position_.X() = msg->pose(i).position().x();
      target_position_.Y() = msg->pose(i).position().y();
      target_position_.Z() = msg->pose(i).position().z();
      got_target_position_ = true;
      return;
    }
  }
}

//}

/* getCamOrientation(...)//{ */

ignition::math::Quaterniond CamSynchronizer::getCamOrientation(const ignition::math::Vector3d& cam_pose) {
  std::scoped_lock lock(mutex_target_position_);
  ignition::math::Matrix4d rotation_matrix = ignition::math::Matrix4d::LookAt(cam_pose, target_position_);
  return rotation_matrix.Rotation();
}

//}

/* msg2position (...)//{ */

ignition::math::Vector3d CamSynchronizer::msg2position(const GazeboPoseConstPtr& msg) {
  ignition::math::Vector3d pos;
  pos.X() = msg->position().x();
  pos.Y() = msg->position().y();
  pos.Z() = msg->position().z();
  return pos;
}

//}

/* pose2transform(...) //{ */

geometry_msgs::Transform CamSynchronizer::pose2transform(const ignition::math::Pose3d& pose) {
  geometry_msgs::Transform tf;
  tf.translation.x = pose.Pos().X();
  tf.translation.y = pose.Pos().Y();
  tf.translation.z = pose.Pos().Z();

  tf.rotation.w = pose.Rot().W();
  tf.rotation.x = pose.Rot().X();
  tf.rotation.y = pose.Rot().Y();
  tf.rotation.z = pose.Rot().Z();
  return tf;
}

//}

/* publishTF(...) //{ */

void CamSynchronizer::publishTF(const ignition::math::Pose3d& cam_pose) {
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = _world_origin_frame_id_;
  tf.header.stamp    = ros::Time::now();
  tf.child_frame_id  = _target_frame_id_;
  tf.transform       = pose2transform(cam_pose);
  broadcaster_->sendTransform(tf);
}

//}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CamSynchronizer)
}  // namespace gazebo
