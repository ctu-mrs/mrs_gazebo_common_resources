#include "mrs_gazebo_common_resources/servo_camera_plugin.hpp"

#include <sstream>
#include <iomanip>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace gz;
using namespace sim;
using namespace systems;

/* ~ServoCameraPlugin */ //{
ServoCameraPlugin::~ServoCameraPlugin() {
  if (executor_) {
    executor_->cancel();
  }

  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}
//}

/* Configure */ //{
void ServoCameraPlugin::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &p_sdf, gz::sim::EntityComponentManager &ecm,
                                  [[maybe_unused]] gz::sim::EventManager &event_mgr) {
  model_       = gz::sim::Model(entity);
  parent_name_ = model_.Name(ecm);

  // initialize ROS
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // the timers run in simulation time, the same as ROS timers did in Gazebo Classic
  node_ = std::make_shared<rclcpp::Node>("servo_camera_plugin", parent_name_, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));

  RCLCPP_INFO(node_->get_logger(), "[%s]: Servo camera plugin started.", node_->get_name());

  /* Load params //{ */
  if (p_sdf->HasElement("parent_frame_name")) {
    parent_frame_name_ = p_sdf->Get<std::string>("parent_frame_name");
  } else {
    parent_frame_name_ = "world";
    RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: parent_frame_name not defined. Setting to default value: %s", parent_name_.c_str(),
                parent_frame_name_.c_str());
  }

  if (p_sdf->HasElement("update_rate")) {
    update_rate_ = p_sdf->Get<double>("update_rate");
  } else {
    update_rate_ = 30;
    RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: update_rate not defined. Setting to default value: %f", parent_name_.c_str(), update_rate_);
  }

  if (p_sdf->HasElement("setpoint_rate")) {
    setpoint_rate_ = p_sdf->Get<double>("setpoint_rate");
  } else {
    setpoint_rate_ = 1;
    RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: setpoint_rate not defined. Setting to default value: %f", parent_name_.c_str(), setpoint_rate_);
  }

  if (p_sdf->HasElement("yaw")) {
    auto yaw_elem = p_sdf->FindElement("yaw");

    if (yaw_elem->HasElement("joint_name"))
      yaw_servo_joint_.name = yaw_elem->Get<std::string>("joint_name");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Joint name yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("parent_link"))
      yaw_servo_joint_.parent_link = yaw_elem->Get<std::string>("parent_link");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Parent link yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("frame_name"))
      yaw_servo_joint_.frame_name = yaw_elem->Get<std::string>("frame_name");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Frame name yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("min_angle"))
      yaw_servo_joint_.min_angle = yaw_elem->Get<double>("min_angle");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Min angle yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("max_angle"))
      yaw_servo_joint_.max_angle = yaw_elem->Get<double>("max_angle");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Max angle yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("max_rate"))
      yaw_servo_joint_.max_rate = yaw_elem->Get<double>("max_rate");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Max rate yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("compensate_tilt"))
      yaw_servo_joint_.compensate_tilt = yaw_elem->Get<bool>("compensate_tilt");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Compensate tilt yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("offset_x"))
      yaw_servo_joint_.offset_x = yaw_elem->Get<double>("offset_x");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset x yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("offset_y"))
      yaw_servo_joint_.offset_y = yaw_elem->Get<double>("offset_y");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset y yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("offset_z"))
      yaw_servo_joint_.offset_z = yaw_elem->Get<double>("offset_z");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset z yaw not defined.", parent_name_.c_str());

  } else {
    RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Yaw joint not defined.", parent_name_.c_str());
  }

  if (p_sdf->HasElement("roll")) {
    auto roll_elem = p_sdf->FindElement("roll");
    if (roll_elem->HasElement("joint_name"))
      roll_servo_joint_.name = roll_elem->Get<std::string>("joint_name");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Joint name roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("parent_link"))
      roll_servo_joint_.parent_link = roll_elem->Get<std::string>("parent_link");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Parent link roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("frame_name"))
      roll_servo_joint_.frame_name = roll_elem->Get<std::string>("frame_name");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Frame name roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("min_angle"))
      roll_servo_joint_.min_angle = roll_elem->Get<double>("min_angle");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Min angle roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("max_angle"))
      roll_servo_joint_.max_angle = roll_elem->Get<double>("max_angle");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Max angle roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("max_rate"))
      roll_servo_joint_.max_rate = roll_elem->Get<double>("max_rate");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Max rate roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("compensate_tilt"))
      roll_servo_joint_.compensate_tilt = roll_elem->Get<bool>("compensate_tilt");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Compensate tilt roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("offset_x"))
      roll_servo_joint_.offset_x = roll_elem->Get<double>("offset_x");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset x roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("offset_y"))
      roll_servo_joint_.offset_y = roll_elem->Get<double>("offset_y");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset y roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("offset_z"))
      roll_servo_joint_.offset_z = roll_elem->Get<double>("offset_z");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset z roll not defined.", parent_name_.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Roll joint not defined.", parent_name_.c_str());
  }

  if (p_sdf->HasElement("pitch")) {
    auto pitch_elem = p_sdf->FindElement("pitch");
    if (pitch_elem->HasElement("joint_name"))
      pitch_servo_joint_.name = pitch_elem->Get<std::string>("joint_name");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Joint name pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("parent_link"))
      pitch_servo_joint_.parent_link = pitch_elem->Get<std::string>("parent_link");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Parent link pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("frame_name"))
      pitch_servo_joint_.frame_name = pitch_elem->Get<std::string>("frame_name");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Frame name pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("min_angle"))
      pitch_servo_joint_.min_angle = pitch_elem->Get<double>("min_angle");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Min angle pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("max_angle"))
      pitch_servo_joint_.max_angle = pitch_elem->Get<double>("max_angle");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Max angle pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("max_rate"))
      pitch_servo_joint_.max_rate = pitch_elem->Get<double>("max_rate");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Max rate pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("compensate_tilt"))
      pitch_servo_joint_.compensate_tilt = pitch_elem->Get<bool>("compensate_tilt");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Compensate tilt pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("offset_x"))
      pitch_servo_joint_.offset_x = pitch_elem->Get<double>("offset_x");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset x pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("offset_y"))
      pitch_servo_joint_.offset_y = pitch_elem->Get<double>("offset_y");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset y pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("offset_z"))
      pitch_servo_joint_.offset_z = pitch_elem->Get<double>("offset_z");
    else
      RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Offset z pitch not defined.", parent_name_.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: Pitch joint not defined.", parent_name_.c_str());
  }
  //}

  // ROS subscriber and service for control camera orientation
  std::stringstream ss;
  ss << "/" << parent_name_ << "/servo_camera/set_orientation";
  camera_orientation_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      ss.str(), 1, std::bind(&ServoCameraPlugin::desiredOrientationCallback, this, std::placeholders::_1));

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/set_orientation";
  camera_orientation_srv_ = node_->create_service<mrs_msgs::srv::Vec4>(
      ss.str(), std::bind(&ServoCameraPlugin::setOrientationCallback, this, std::placeholders::_1, std::placeholders::_2));

  // ROS publisher for camera orientation
  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/orientation";
  camera_orientation_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(ss.str(), 1);

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/orientation_setpoint";
  camera_setpoint_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(ss.str(), 1);

  // ROS services for triggering compensation
  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/compensate_tilt_pitch";
  tilt_compensation_pitch_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      ss.str(), std::bind(&ServoCameraPlugin::triggerTiltCompensationPitchCallback, this, std::placeholders::_1, std::placeholders::_2));

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/compensate_tilt_roll";
  tilt_compensation_roll_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      ss.str(), std::bind(&ServoCameraPlugin::triggerTiltCompensationRollCallback, this, std::placeholders::_1, std::placeholders::_2));

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/compensate_tilt_yaw";
  tilt_compensation_yaw_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      ss.str(), std::bind(&ServoCameraPlugin::triggerTiltCompensationYawCallback, this, std::placeholders::_1, std::placeholders::_2));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Timers
  camera_setpoint_timer_ = node_->create_timer(std::chrono::duration<double>(1.0 / setpoint_rate_), [this]() -> void { publishCameraSetpoint(); });
  camera_timer_          = node_->create_timer(std::chrono::duration<double>(1.0 / update_rate_), std::bind(&ServoCameraPlugin::cameraTimerCallback, this));

  // spin the node in a separate thread, gazebo does not do it for us
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Servo camera initialized.", parent_name_.c_str());
}
//}

/* cameraTimerCallback */ //{
void ServoCameraPlugin::cameraTimerCallback() {
  if (!initialized_)
    return;

  moveCamera();
  publishCameraOrientation();
  publishTF();
}
//}

/* publishCameraSetpoint() //{ */
void ServoCameraPlugin::publishCameraSetpoint() {
  std::scoped_lock lock(mutex_orientation_setpoint_);

  std_msgs::msg::Float32MultiArray msg;

  std_msgs::msg::MultiArrayLayout layout;
  layout.dim.resize(1);
  layout.dim[0].label  = "RPY";
  layout.dim[0].size   = 3;
  layout.dim[0].stride = 3;

  msg.layout = layout;
  msg.data.push_back(roll_setpoint_);
  msg.data.push_back(pitch_setpoint_);
  msg.data.push_back(yaw_setpoint_);

  camera_setpoint_pub_->publish(msg);
}
//}

/* desiredOrientationCallback */ //{
void ServoCameraPlugin::desiredOrientationCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() > 3 || msg->data.size() < 1) {
    RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: cameraOrientationCallback: Invalid request. Unexpected size of data.", parent_name_.c_str());
    return;
  }

  std::scoped_lock lock(mutex_orientation_setpoint_);

  roll_setpoint_ = fmin(fmax(roll_servo_joint_.min_angle, msg->data[0]), roll_servo_joint_.max_angle); // clip value in limits
  roll_setpoint_ = getBoundedAngle(roll_setpoint_);
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name_.c_str(), roll_setpoint_,
              msg->data[0]);

  if (msg->data.size() > 1) {
    pitch_setpoint_ = fmin(fmax(pitch_servo_joint_.min_angle, msg->data[1]), pitch_servo_joint_.max_angle); // clip value in limits
    pitch_setpoint_ = getBoundedAngle(pitch_setpoint_);
    RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name_.c_str(), pitch_setpoint_,
                msg->data[1]);
  }

  if (msg->data.size() > 2) {
    yaw_setpoint_ = fmin(fmax(yaw_servo_joint_.min_angle, msg->data[2]), yaw_servo_joint_.max_angle); // clip value in limits
    yaw_setpoint_ = getBoundedAngle(yaw_setpoint_);
    RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Change camera yaw to %.2f! Requested unlimited yaw = %.2f", parent_name_.c_str(), yaw_setpoint_,
                msg->data[2]);
  }

  // Publish the desired orientation
  publishCameraSetpoint();
}
//}

/* setOrientationCallback() //{ */
void ServoCameraPlugin::setOrientationCallback(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> req, std::shared_ptr<mrs_msgs::srv::Vec4::Response> res) {
  if (req->goal[3] != 0) {
    res->success = false;
    res->message = "Camera orientation message received with non-zero value in 4th element.";

    RCLCPP_WARN(node_->get_logger(), "[%s][Servo camera]: %s", parent_name_.c_str(), res->message.c_str());
    return;
  }

  double new_desired_roll = fmin(fmax(roll_servo_joint_.min_angle, req->goal[0]), roll_servo_joint_.max_angle); // clip value in limits
  new_desired_roll        = getBoundedAngle(new_desired_roll);
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name_.c_str(), new_desired_roll,
              req->goal[0]);

  double new_desired_pitch = fmin(fmax(pitch_servo_joint_.min_angle, req->goal[1]), pitch_servo_joint_.max_angle); // clip value in limits
  new_desired_pitch        = getBoundedAngle(new_desired_pitch);
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name_.c_str(), new_desired_pitch,
              req->goal[1]);

  double new_desired_yaw = fmin(fmax(yaw_servo_joint_.min_angle, req->goal[2]), yaw_servo_joint_.max_angle); // clip value in limits
  new_desired_yaw        = getBoundedAngle(new_desired_yaw);
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Change camera yaw to %.2f! Requested unlimited yaw = %.2f", parent_name_.c_str(), new_desired_yaw,
              req->goal[2]);

  { // Lock mutex just for the desired orientation assignment
    std::scoped_lock lock(mutex_orientation_setpoint_);

    roll_setpoint_  = new_desired_roll;
    pitch_setpoint_ = new_desired_pitch;
    yaw_setpoint_   = new_desired_yaw;
  }

  std::stringstream ss;
  ss << "Camera orientation processed. " << std::fixed << std::setprecision(2) << "Roll: requested = " << req->goal[0] << ", limited = " << roll_setpoint_
     << "; "
     << "Pitch: requested = " << req->goal[1] << ", limited = " << pitch_setpoint_ << "; "
     << "Yaw: requested = " << req->goal[2] << ", limited = " << yaw_setpoint_ << ".";

  res->success = true;
  res->message = ss.str();

  publishCameraSetpoint();
}
//}

/* triggerTiltCompensationPitchCallback() */ //{
void ServoCameraPlugin::triggerTiltCompensationPitchCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                             std::shared_ptr<std_srvs::srv::SetBool::Response>      res) {
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Trigger tilt compensation pitch!", parent_name_.c_str());

  pitch_servo_joint_.compensate_tilt = req->data;
  res->message                       = pitch_servo_joint_.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  RCLCPP_WARN(node_->get_logger(), "[%s][Servo Camera]: %s.", parent_name_.c_str(), res->message.c_str());

  res->success = true;
}
//}

/* triggerTiltCompensationRollCallback() */ //{
void ServoCameraPlugin::triggerTiltCompensationRollCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                            std::shared_ptr<std_srvs::srv::SetBool::Response>      res) {
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Trigger tilt compensation roll!", parent_name_.c_str());

  roll_servo_joint_.compensate_tilt = req->data;
  res->message                      = roll_servo_joint_.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  RCLCPP_WARN(node_->get_logger(), "[%s][Servo Camera]: %s.", parent_name_.c_str(), res->message.c_str());

  res->success = true;
}
//}

/* triggerTiltCompensationYawCallback() */ //{
void ServoCameraPlugin::triggerTiltCompensationYawCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                           std::shared_ptr<std_srvs::srv::SetBool::Response>      res) {
  RCLCPP_INFO(node_->get_logger(), "[%s][Servo camera]: Trigger tilt compensation yaw!", parent_name_.c_str());

  yaw_servo_joint_.compensate_tilt = req->data;
  res->message                     = yaw_servo_joint_.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  RCLCPP_WARN(node_->get_logger(), "[%s][Servo Camera]: %s.", parent_name_.c_str(), res->message.c_str());

  res->success = true;
}
//}

/* getBoundedAngle(double angle) //{ */
double ServoCameraPlugin::getBoundedAngle(double angle) {
  return angle > M_PI ? -2 * M_PI + angle : angle < -M_PI ? 2 * M_PI + angle : angle;
}
//}

/* moveCamera() //{ */
void ServoCameraPlugin::moveCamera() {
  std::scoped_lock lock(mutex_orientation_setpoint_, mutex_model_orientation_);

  // get desired pitch and roll angles compensated for current tilts of model_
  double compensated_pitch = pitch_servo_joint_.compensate_tilt ? pitch_setpoint_ - model_pitch_ : pitch_setpoint_;
  double compensated_roll  = roll_servo_joint_.compensate_tilt ? roll_setpoint_ - model_roll_ : roll_setpoint_;
  double compensated_yaw   = yaw_servo_joint_.compensate_tilt ? yaw_setpoint_ - model_yaw_ : yaw_setpoint_;

  // find required angle in next step that satisfies limits on angular_rates
  double abs_diff_pitch = abs(offset_pitch_ - compensated_pitch);
  double min_diff_pitch = abs_diff_pitch > M_PI ? 2 * M_PI - abs_diff_pitch : abs_diff_pitch;

  if (abs(min_diff_pitch) > 1e-4) {
    double dir = compensated_pitch > offset_pitch_ ? 1.0 : -1.0;
    dir        = abs(min_diff_pitch - abs_diff_pitch) > 1e-4 ? -1.0 * dir : dir;
    offset_pitch_ += dir * fmin(abs(min_diff_pitch), pitch_servo_joint_.max_rate / update_rate_);
    offset_pitch_ = getBoundedAngle(offset_pitch_);
  }

  double abs_diff_roll = abs(offset_roll_ - compensated_roll);
  double min_diff_roll = abs_diff_roll > M_PI ? 2 * M_PI - abs_diff_roll : abs_diff_roll;

  if (abs(min_diff_roll) > 1e-4) {
    double dir = compensated_roll > offset_roll_ ? 1.0 : -1.0;
    dir        = abs(min_diff_roll - abs_diff_roll) > 1e-4 ? -1.0 * dir : dir;
    offset_roll_ += dir * fmin(abs(min_diff_roll), roll_servo_joint_.max_rate / update_rate_);
    offset_roll_ = getBoundedAngle(offset_roll_);
  }

  double abs_diff_yaw = abs(offset_yaw_ - compensated_yaw);
  double min_diff_yaw = abs_diff_yaw > M_PI ? 2 * M_PI - abs_diff_yaw : abs_diff_yaw;

  if (abs(min_diff_yaw) > 1e-4) {
    double dir = compensated_yaw > offset_yaw_ ? 1.0 : -1.0;
    dir        = abs(min_diff_yaw - abs_diff_yaw) > 1e-4 ? -1.0 * dir : dir;
    offset_yaw_ += dir * fmin(abs(min_diff_yaw), yaw_servo_joint_.max_rate / update_rate_);
    offset_yaw_ = getBoundedAngle(offset_yaw_);
  }

  // the joint coordinates are set in PreUpdate(), the entity component manager is not accessible from here
}
//}

/* setJointPosition() //{ */
void ServoCameraPlugin::setJointPosition(gz::sim::EntityComponentManager &ecm, const ServoJoint &servo_joint, const double position) {
  gz::sim::Entity joint = model_.JointByName(ecm, servo_joint.name);

  if (joint == gz::sim::kNullEntity) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[%s]: Servo camera joint %s did not found.", node_->get_name(),
                         servo_joint.name.c_str());
    return;
  }

  ecm.SetComponentData<gz::sim::components::JointPositionReset>(joint, {position});
  ecm.SetComponentData<gz::sim::components::JointVelocityReset>(joint, {0.0});
}
//}

/* publishCameraOrientation() //{ */
void ServoCameraPlugin::publishCameraOrientation() {
  std::scoped_lock lock(mutex_orientation_setpoint_);

  std_msgs::msg::Float32MultiArray msg;

  std_msgs::msg::MultiArrayLayout layout;
  layout.dim.resize(1);
  layout.dim[0].label  = "RPY";
  layout.dim[0].size   = 3;
  layout.dim[0].stride = 3;

  msg.layout = layout;
  msg.data.push_back(offset_roll_);
  msg.data.push_back(offset_pitch_);
  msg.data.push_back(offset_yaw_);

  camera_orientation_pub_->publish(msg);
}
//}

/* publishTF() //{ */
void ServoCameraPlugin::publishTF() {
  std::scoped_lock lock(mutex_model_orientation_);

  // Yaw
  geometry_msgs::msg::TransformStamped transform_yaw;
  transform_yaw.header.stamp    = node_->now();
  transform_yaw.header.frame_id = parent_frame_name_;
  transform_yaw.child_frame_id  = yaw_servo_joint_.frame_name;

  transform_yaw.transform.translation.x = yaw_servo_joint_.offset_x;
  transform_yaw.transform.translation.y = yaw_servo_joint_.offset_y;
  transform_yaw.transform.translation.z = yaw_servo_joint_.offset_z;

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0, 0, offset_yaw_);
  transform_yaw.transform.rotation = tf2::toMsg(q_yaw);

  // Roll
  geometry_msgs::msg::TransformStamped transform_roll;
  transform_roll.header.stamp    = node_->now();
  transform_roll.header.frame_id = yaw_servo_joint_.frame_name;
  transform_roll.child_frame_id  = roll_servo_joint_.frame_name;

  transform_roll.transform.translation.x = roll_servo_joint_.offset_x;
  transform_roll.transform.translation.y = roll_servo_joint_.offset_y;
  transform_roll.transform.translation.z = roll_servo_joint_.offset_z;

  tf2::Quaternion q_roll;
  q_roll.setRPY(offset_roll_, 0, 0);
  transform_roll.transform.rotation = tf2::toMsg(q_roll);

  // Pitch
  geometry_msgs::msg::TransformStamped transform_pitch;
  transform_pitch.header.stamp    = node_->now();
  transform_pitch.header.frame_id = roll_servo_joint_.frame_name;
  transform_pitch.child_frame_id  = pitch_servo_joint_.frame_name;

  transform_pitch.transform.translation.x = pitch_servo_joint_.offset_x;
  transform_pitch.transform.translation.y = pitch_servo_joint_.offset_y;
  transform_pitch.transform.translation.z = pitch_servo_joint_.offset_z;

  tf2::Quaternion q_pitch;
  q_pitch.setRPY(0, offset_pitch_, 0);
  transform_pitch.transform.rotation = tf2::toMsg(q_pitch);

  // Publish the transforms
  tf_broadcaster_->sendTransform({transform_yaw, transform_roll, transform_pitch});
}
//}

/* PreUpdate //{ */
void ServoCameraPlugin::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) {
  if (!initialized_ || info.paused)
    return;

  // get tilts of model
  RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: Updating model orientation.", node_->get_name());

  {
    std::scoped_lock lock(mutex_model_orientation_);

    gz::math::Pose3d model_pose = gz::sim::worldPose(model_.Entity(), ecm);
    model_roll_                 = model_pose.Rot().Roll();
    model_pitch_                = model_pose.Rot().Pitch();
    model_yaw_                  = model_pose.Rot().Yaw();
  }

  // set joint coordinates based on computed required orientations of links
  // (in every step, otherwise the links would fall down with gravity between the timer updates)
  std::scoped_lock lock(mutex_orientation_setpoint_);

  setJointPosition(ecm, roll_servo_joint_, offset_roll_);
  setJointPosition(ecm, pitch_servo_joint_, offset_pitch_);
  setJointPosition(ecm, yaw_servo_joint_, offset_yaw_);
}
//}

GZ_ADD_PLUGIN(ServoCameraPlugin, System, ServoCameraPlugin::ISystemConfigure, ServoCameraPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ServoCameraPlugin, "gz::sim::systems::ServoCameraPlugin")
