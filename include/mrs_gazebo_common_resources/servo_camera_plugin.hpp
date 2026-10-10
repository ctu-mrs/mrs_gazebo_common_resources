#ifndef MRS_GZ_SIM_SYSTEMS_SERVOCAMERAPLUGIN_HH_
#define MRS_GZ_SIM_SYSTEMS_SERVOCAMERAPLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mrs_msgs/srv/vec4.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

/// \brief Parameters and state of one servo axis
struct ServoJoint
{
  std::string name;
  std::string parent_link;
  std::string frame_name;
  double      min_angle;
  double      max_angle;
  double      max_rate;
  bool        compensate_tilt;
  double      offset_x;
  double      offset_y;
  double      offset_z;
};

/// \brief Rotates the yaw, roll and pitch joints of a camera gimbal towards an orientation setpoint received over ROS,
/// with rate and angle limits and optional compensation of the vehicle tilt. Publishes the TF of the gimbal.
/// Port of the Gazebo Classic ServoCameraPlugin.
class ServoCameraPlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
public:
  ~ServoCameraPlugin() override;

  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &p_sdf, gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &event_mgr) override;
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override;

private:
  gz::sim::Model model_{gz::sim::kNullEntity};

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread                                          executor_thread_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tilt_compensation_pitch_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tilt_compensation_roll_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tilt_compensation_yaw_srv_;

  rclcpp::Service<mrs_msgs::srv::Vec4>::SharedPtr                   camera_orientation_srv_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr camera_orientation_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    camera_orientation_pub_;

  rclcpp::TimerBase::SharedPtr                                   camera_timer_;
  rclcpp::TimerBase::SharedPtr                                   camera_setpoint_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr camera_setpoint_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void desiredOrientationCallback(const std_msgs::msg::Float32MultiArray::SharedPtr desired_orientation);
  void setOrientationCallback(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> req, std::shared_ptr<mrs_msgs::srv::Vec4::Response> res);

  void triggerTiltCompensationPitchCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void triggerTiltCompensationRollCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void triggerTiltCompensationYawCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  void   publishTF();
  void   publishCameraOrientation();
  void   cameraTimerCallback();
  void   publishCameraSetpoint();
  void   moveCamera();
  void   setJointPosition(gz::sim::EntityComponentManager &ecm, const ServoJoint &servo_joint, const double position);
  double getBoundedAngle(double angle);

  // camera and joint names
  std::string parent_name_;
  std::string parent_frame_name_;

  ServoJoint roll_servo_joint_;
  ServoJoint pitch_servo_joint_;
  ServoJoint yaw_servo_joint_;

  // camera offset with respect to model
  double offset_roll_  = 0.0;
  double offset_pitch_ = 0.0;
  double offset_yaw_   = 0.0;

  double update_rate_;    // update rate of camera position
  double setpoint_rate_;  // rate of publishing setpoint orientation

  std::recursive_mutex mutex_orientation_setpoint_;
  double               pitch_setpoint_ = 0.0;
  double               roll_setpoint_  = 0.0;
  double               yaw_setpoint_   = 0.0;

  // model orientation in world frame
  std::mutex mutex_model_orientation_;
  double     model_roll_  = 0.0;
  double     model_pitch_ = 0.0;
  double     model_yaw_   = 0.0;

  bool initialized_ = false;
};

}
}
}
}

#endif
