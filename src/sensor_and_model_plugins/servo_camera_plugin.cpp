#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/physics/Model.hh>

#include <random>
#include <stdio.h>
#include <stdexcept>
#include <sdf/sdf.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec4.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ignition/math.hh>

namespace gazebo
{

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

/* Class definition */  //{
class ServoCameraPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr p_sdf);

private:
  physics::ModelPtr model_;
  ros::NodeHandle  *nh_;

  ros::ServiceServer tilt_compensation_pitch_srv_;
  ros::ServiceServer tilt_compensation_roll_srv_;
  ros::ServiceServer tilt_compensation_yaw_srv_;

  ros::ServiceServer camera_orientation_srv_;
  ros::Subscriber    camera_orientation_sub_;
  ros::Publisher     camera_orientation_pub_;

  ros::Timer     camera_timer_;
  ros::Timer     camera_setpoint_timer_;
  ros::Publisher camera_setpoint_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  void desiredOrientationCallback(const std_msgs::Float32MultiArray &desired_orientation);
  bool setOrientationCallback(mrs_msgs::Vec4Request &req, mrs_msgs::Vec4Response &res);

  bool triggerTiltCompensationPitchCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool triggerTiltCompensationRollCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool triggerTiltCompensationYawCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  void   publishTF();
  void   publishCameraOrientation();
  void   OnUpdate();
  void   cameraTimerCallback(const ros::TimerEvent &event);
  void   publishCameraSetpoint();
  void   moveCamera();
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
  double               pitch_setpoint_;
  double               roll_setpoint_;
  double               yaw_setpoint_;

  // model orientation in world frame
  std::mutex mutex_model_orientation_;
  double     model_roll_;
  double     model_pitch_;
  double     model_yaw_;

  bool initialized_ = false;

  event::ConnectionPtr update_connection_;
};
//}

/* Load */  //{
void ServoCameraPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr p_sdf) {
  ROS_INFO("[%s]: Servo camera plugin started.", ros::this_node::getName().c_str());
  model_       = parent;
  parent_name_ = model_->GetName().c_str();

  /* Load params //{ */
  if (p_sdf->HasElement("parent_frame_name")) {
    parent_frame_name_ = p_sdf->Get<std::string>("parent_frame_name");
  } else {
    parent_frame_name_ = "world";
    ROS_WARN("[%s][Servo camera]: parent_frame_name not defined. Setting to default value: %s", parent_name_.c_str(), parent_frame_name_.c_str());
  }

  if (p_sdf->HasElement("update_rate")) {
    update_rate_ = p_sdf->Get<double>("update_rate");
  } else {
    update_rate_ = 30;
    ROS_WARN("[%s][Servo camera]: update_rate not defined. Setting to default value: %f", parent_name_.c_str(), update_rate_);
  }

  if (p_sdf->HasElement("setpoint_rate")) {
    setpoint_rate_ = p_sdf->Get<double>("setpoint_rate");
  } else {
    setpoint_rate_ = 1;
    ROS_WARN("[%s][Servo camera]: setpoint_rate not defined. Setting to default value: %f", parent_name_.c_str(), setpoint_rate_);
  }

  if (p_sdf->HasElement("yaw")) {
    auto yaw_elem = p_sdf->GetElement("yaw");

    if (yaw_elem->HasElement("joint_name"))
      yaw_servo_joint_.name = yaw_elem->Get<std::string>("joint_name");
    else
      ROS_WARN("[%s][Servo camera]: Joint name yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("parent_link"))
      yaw_servo_joint_.parent_link = yaw_elem->Get<std::string>("parent_link");
    else
      ROS_WARN("[%s][Servo camera]: Parent link yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("frame_name"))
      yaw_servo_joint_.frame_name = yaw_elem->Get<std::string>("frame_name");
    else
      ROS_WARN("[%s][Servo camera]: Frame name yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("min_angle"))
      yaw_servo_joint_.min_angle = yaw_elem->Get<double>("min_angle");
    else
      ROS_WARN("[%s][Servo camera]: Min angle yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("max_angle"))
      yaw_servo_joint_.max_angle = yaw_elem->Get<double>("max_angle");
    else
      ROS_WARN("[%s][Servo camera]: Max angle yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("max_rate"))
      yaw_servo_joint_.max_rate = yaw_elem->Get<double>("max_rate");
    else
      ROS_WARN("[%s][Servo camera]: Max rate yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("compensate_tilt"))
      yaw_servo_joint_.compensate_tilt = yaw_elem->Get<bool>("compensate_tilt");
    else
      ROS_WARN("[%s][Servo camera]: Compensate tilt yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("offset_x"))
      yaw_servo_joint_.offset_x = yaw_elem->Get<double>("offset_x");
    else
      ROS_WARN("[%s][Servo camera]: Offset x yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("offset_y"))
      yaw_servo_joint_.offset_y = yaw_elem->Get<double>("offset_y");
    else
      ROS_WARN("[%s][Servo camera]: Offset y yaw not defined.", parent_name_.c_str());

    if (yaw_elem->HasElement("offset_z"))
      yaw_servo_joint_.offset_z = yaw_elem->Get<double>("offset_z");
    else
      ROS_WARN("[%s][Servo camera]: Offset z yaw not defined.", parent_name_.c_str());

  } else {
    ROS_WARN("[%s][Servo camera]: Yaw joint not defined.", parent_name_.c_str());
  }

  if (p_sdf->HasElement("roll")) {
    auto roll_elem = p_sdf->GetElement("roll");
    if (roll_elem->HasElement("joint_name"))
      roll_servo_joint_.name = roll_elem->Get<std::string>("joint_name");
    else
      ROS_WARN("[%s][Servo camera]: Joint name roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("parent_link"))
      roll_servo_joint_.parent_link = roll_elem->Get<std::string>("parent_link");
    else
      ROS_WARN("[%s][Servo camera]: Parent link roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("frame_name"))
      roll_servo_joint_.frame_name = roll_elem->Get<std::string>("frame_name");
    else
      ROS_WARN("[%s][Servo camera]: Frame name roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("min_angle"))
      roll_servo_joint_.min_angle = roll_elem->Get<double>("min_angle");
    else
      ROS_WARN("[%s][Servo camera]: Min angle roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("max_angle"))
      roll_servo_joint_.max_angle = roll_elem->Get<double>("max_angle");
    else
      ROS_WARN("[%s][Servo camera]: Max angle roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("max_rate"))
      roll_servo_joint_.max_rate = roll_elem->Get<double>("max_rate");
    else
      ROS_WARN("[%s][Servo camera]: Max rate roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("compensate_tilt"))
      roll_servo_joint_.compensate_tilt = roll_elem->Get<bool>("compensate_tilt");
    else
      ROS_WARN("[%s][Servo camera]: Compensate tilt roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("offset_x"))
      roll_servo_joint_.offset_x = roll_elem->Get<double>("offset_x");
    else
      ROS_WARN("[%s][Servo camera]: Offset x roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("offset_y"))
      roll_servo_joint_.offset_y = roll_elem->Get<double>("offset_y");
    else
      ROS_WARN("[%s][Servo camera]: Offset y roll not defined.", parent_name_.c_str());

    if (roll_elem->HasElement("offset_z"))
      roll_servo_joint_.offset_z = roll_elem->Get<double>("offset_z");
    else
      ROS_WARN("[%s][Servo camera]: Offset z roll not defined.", parent_name_.c_str());
  } else {
    ROS_WARN("[%s][Servo camera]: Roll joint not defined.", parent_name_.c_str());
  }

  if (p_sdf->HasElement("pitch")) {
    auto pitch_elem = p_sdf->GetElement("pitch");
    if (pitch_elem->HasElement("joint_name"))
      pitch_servo_joint_.name = pitch_elem->Get<std::string>("joint_name");
    else
      ROS_WARN("[%s][Servo camera]: Joint name pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("parent_link"))
      pitch_servo_joint_.parent_link = pitch_elem->Get<std::string>("parent_link");
    else
      ROS_WARN("[%s][Servo camera]: Parent link pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("frame_name"))
      pitch_servo_joint_.frame_name = pitch_elem->Get<std::string>("frame_name");
    else
      ROS_WARN("[%s][Servo camera]: Frame name pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("min_angle"))
      pitch_servo_joint_.min_angle = pitch_elem->Get<double>("min_angle");
    else
      ROS_WARN("[%s][Servo camera]: Min angle pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("max_angle"))
      pitch_servo_joint_.max_angle = pitch_elem->Get<double>("max_angle");
    else
      ROS_WARN("[%s][Servo camera]: Max angle pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("max_rate"))
      pitch_servo_joint_.max_rate = pitch_elem->Get<double>("max_rate");
    else
      ROS_WARN("[%s][Servo camera]: Max rate pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("compensate_tilt"))
      pitch_servo_joint_.compensate_tilt = pitch_elem->Get<bool>("compensate_tilt");
    else
      ROS_WARN("[%s][Servo camera]: Compensate tilt pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("offset_x"))
      pitch_servo_joint_.offset_x = pitch_elem->Get<double>("offset_x");
    else
      ROS_WARN("[%s][Servo camera]: Offset x pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("offset_y"))
      pitch_servo_joint_.offset_y = pitch_elem->Get<double>("offset_y");
    else
      ROS_WARN("[%s][Servo camera]: Offset y pitch not defined.", parent_name_.c_str());

    if (pitch_elem->HasElement("offset_z"))
      pitch_servo_joint_.offset_z = pitch_elem->Get<double>("offset_z");
    else
      ROS_WARN("[%s][Servo camera]: Offset z pitch not defined.", parent_name_.c_str());
  } else {
    ROS_WARN("[%s][Servo camera]: Pitch joint not defined.", parent_name_.c_str());
  }
  //}

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "servo_camera_plugin", ros::init_options::NoSigintHandler);
  nh_ = new ros::NodeHandle("~");

  // ROS subscriber and service for control camera orientation
  std::stringstream ss;
  ss << "/" << parent_name_ << "/servo_camera/set_orientation";
  camera_orientation_sub_ = nh_->subscribe(ss.str().c_str(), 1, &ServoCameraPlugin::desiredOrientationCallback, this);

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/set_orientation";
  camera_orientation_srv_ = nh_->advertiseService(ss.str().c_str(), &ServoCameraPlugin::setOrientationCallback, this);

  // ROS publisher for camera orientation
  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/orientation";
  camera_orientation_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(ss.str().c_str(), 1);

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/orientation_setpoint";
  camera_setpoint_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(ss.str().c_str(), 1);

  // ROS services for triggering compensation
  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/compensate_tilt_pitch";
  tilt_compensation_pitch_srv_ = nh_->advertiseService(ss.str().c_str(), &ServoCameraPlugin::triggerTiltCompensationPitchCallback, this);

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/compensate_tilt_roll";
  tilt_compensation_roll_srv_ = nh_->advertiseService(ss.str().c_str(), &ServoCameraPlugin::triggerTiltCompensationRollCallback, this);

  ss.str(std::string());
  ss << "/" << parent_name_ << "/servo_camera/compensate_tilt_yaw";
  tilt_compensation_yaw_srv_ = nh_->advertiseService(ss.str().c_str(), &ServoCameraPlugin::triggerTiltCompensationYawCallback, this);

  // Timers
  camera_setpoint_timer_ = nh_->createTimer(ros::Duration(1.0 / setpoint_rate_), [this]([[maybe_unused]] const ros::TimerEvent &event) -> void { publishCameraSetpoint(); });
  camera_timer_          = nh_->createTimer(ros::Rate(update_rate_), &ServoCameraPlugin::cameraTimerCallback, this);
  update_connection_     = event::Events::ConnectWorldUpdateBegin(std::bind(&ServoCameraPlugin::OnUpdate, this));

  initialized_ = true;
  ROS_INFO("[%s][Servo camera]: Servo camera initialized.", parent_name_.c_str());
}
//}

/* cameraTimerCallback */  //{
void ServoCameraPlugin::cameraTimerCallback(const ros::TimerEvent &event) {
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

  std_msgs::Float32MultiArray msg;

  std_msgs::MultiArrayLayout layout;
  layout.dim.resize(1);
  layout.dim[0].label  = "RPY";
  layout.dim[0].size   = 3;
  layout.dim[0].stride = 3;

  msg.layout = layout;
  msg.data.push_back(roll_setpoint_);
  msg.data.push_back(pitch_setpoint_);
  msg.data.push_back(yaw_setpoint_);

  camera_setpoint_pub_.publish(msg);
}
//}

/* desiredOrientationCallback */  //{
void ServoCameraPlugin::desiredOrientationCallback(const std_msgs::Float32MultiArray &msg) {
  if (msg.data.size() > 3 || msg.data.size() < 1) {
    ROS_INFO("[%s][Servo camera]: cameraOrientationCallback: Invalid request. Unexpected size of data.", parent_name_.c_str());
    return;
  }

  std::scoped_lock lock(mutex_orientation_setpoint_);

  roll_setpoint_ = fmin(fmax(roll_servo_joint_.min_angle, msg.data[0]), roll_servo_joint_.max_angle);  // clip value in limits
  roll_setpoint_ = getBoundedAngle(roll_setpoint_);
  ROS_INFO("[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name_.c_str(), roll_setpoint_, msg.data[0]);

  if (msg.data.size() > 1) {
    pitch_setpoint_ = fmin(fmax(pitch_servo_joint_.min_angle, msg.data[1]), pitch_servo_joint_.max_angle);  // clip value in limits
    pitch_setpoint_ = getBoundedAngle(pitch_setpoint_);
    ROS_INFO("[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name_.c_str(), pitch_setpoint_, msg.data[1]);
  }

  if (msg.data.size() > 2) {
    yaw_setpoint_ = fmin(fmax(yaw_servo_joint_.min_angle, msg.data[2]), yaw_servo_joint_.max_angle);  // clip value in limits
    yaw_setpoint_ = getBoundedAngle(yaw_setpoint_);
    ROS_INFO("[%s][Servo camera]: Change camera yaw to %.2f! Requested unlimited yaw = %.2f", parent_name_.c_str(), yaw_setpoint_, msg.data[2]);
  }

  // Publish the desired orientation
  publishCameraSetpoint();
}
//}

/* setOrientationCallback() //{ */
bool ServoCameraPlugin::setOrientationCallback(mrs_msgs::Vec4Request &req, mrs_msgs::Vec4Response &res) {
  if (req.goal[3] != 0) {
    res.success = false;
    res.message = "Camera orientation message received with non-zero value in 4th element.";

    ROS_WARN("[%s][Servo camera]: %s", parent_name_.c_str(), res.message.c_str());
    return true;
  }

  double new_desired_roll = fmin(fmax(roll_servo_joint_.min_angle, req.goal[0]), roll_servo_joint_.max_angle);  // clip value in limits
  new_desired_roll        = getBoundedAngle(new_desired_roll);
  ROS_INFO("[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name_.c_str(), new_desired_roll, req.goal[0]);

  double new_desired_pitch = fmin(fmax(pitch_servo_joint_.min_angle, req.goal[1]), pitch_servo_joint_.max_angle);  // clip value in limits
  new_desired_pitch        = getBoundedAngle(new_desired_pitch);
  ROS_INFO("[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name_.c_str(), new_desired_pitch, req.goal[1]);

  double new_desired_yaw = fmin(fmax(yaw_servo_joint_.min_angle, req.goal[2]), yaw_servo_joint_.max_angle);  // clip value in limits
  new_desired_yaw        = getBoundedAngle(new_desired_yaw);
  ROS_INFO("[%s][Servo camera]: Change camera yaw to %.2f! Requested unlimited yaw = %.2f", parent_name_.c_str(), new_desired_yaw, req.goal[2]);

  {  // Lock mutex just for the desired orientation assignment
    std::scoped_lock lock(mutex_orientation_setpoint_);

    roll_setpoint_  = new_desired_roll;
    pitch_setpoint_ = new_desired_pitch;
    yaw_setpoint_   = new_desired_yaw;
  }

  std::stringstream ss;
  ss << "Camera orientation processed. " << std::fixed << std::setprecision(2)
    << "Roll: requested = " << req.goal[0] << ", limited = " << roll_setpoint_ << "; "
    << "Pitch: requested = " << req.goal[1] << ", limited = " << pitch_setpoint_ << "; "
    << "Yaw: requested = " << req.goal[2] << ", limited = " << yaw_setpoint_ << ".";

  res.success = true;
  res.message = ss.str();

  publishCameraSetpoint();
  return true;
}
//}

/* triggerTiltCompensationPitchCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationPitchCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation pitch!", parent_name_.c_str());

  pitch_servo_joint_.compensate_tilt = req.data;
  res.message                        = pitch_servo_joint_.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name_.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* triggerTiltCompensationRollCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationRollCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation roll!", parent_name_.c_str());

  roll_servo_joint_.compensate_tilt = req.data;
  res.message                       = roll_servo_joint_.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name_.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* triggerTiltCompensationYawCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationYawCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation yaw!", parent_name_.c_str());

  yaw_servo_joint_.compensate_tilt = req.data;
  res.message                      = yaw_servo_joint_.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name_.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* getBoundedAngle(double angle) //{ */
double ServoCameraPlugin::getBoundedAngle(double angle) {
  return angle > M_PI ? -2 * M_PI + angle : angle < -M_PI ? 2 * M_PI + angle : angle;
}
//}

/* moveCamera() //{ */
void ServoCameraPlugin::moveCamera() {
  {
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
  }

  // set joint coordinates based on computed required orientations of links
  if (model_->GetLink(roll_servo_joint_.parent_link) != NULL) {
    if (model_->GetLink(roll_servo_joint_.parent_link)->GetChildJoints().size() != 0) {
      int index_roll = -1;
      for (size_t i = 0; i < model_->GetLink(roll_servo_joint_.parent_link)->GetChildJoints().size(); i++) {
        if (model_->GetLink(roll_servo_joint_.parent_link)->GetChildJoints()[i]->GetName() == roll_servo_joint_.name) {
          index_roll = i;
          break;
        }
      }

      if (index_roll != -1) {
        model_->GetLink(roll_servo_joint_.parent_link)->GetChildJoints()[index_roll]->SetPosition(0, offset_roll_);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera roll joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), roll_servo_joint_.parent_link.c_str());
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: model has no link %s", ros::this_node::getName().c_str(), roll_servo_joint_.parent_link.c_str());
  }

  if (model_->GetLink(pitch_servo_joint_.parent_link) != NULL) {
    if (model_->GetLink(pitch_servo_joint_.parent_link)->GetChildJoints().size() != 0) {
      int index_pitch = -1;
      for (size_t i = 0; i < model_->GetLink(pitch_servo_joint_.parent_link)->GetChildJoints().size(); i++) {
        if (model_->GetLink(pitch_servo_joint_.parent_link)->GetChildJoints()[i]->GetName() == pitch_servo_joint_.name) {
          index_pitch = i;
          break;
        }
      }

      if (index_pitch != -1) {
        model_->GetLink(pitch_servo_joint_.parent_link)->GetChildJoints()[index_pitch]->SetPosition(0, offset_pitch_);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera pitch joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), pitch_servo_joint_.parent_link.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: model has no link %s", ros::this_node::getName().c_str(), pitch_servo_joint_.parent_link.c_str());
  }

  if (model_->GetLink(yaw_servo_joint_.parent_link) != NULL) {
    if (model_->GetLink(yaw_servo_joint_.parent_link)->GetChildJoints().size() != 0) {
      int index_yaw = -1;
      for (size_t i = 0; i < model_->GetLink(yaw_servo_joint_.parent_link)->GetChildJoints().size(); i++) {
        if (model_->GetLink(yaw_servo_joint_.parent_link)->GetChildJoints()[i]->GetName() == yaw_servo_joint_.name) {
          index_yaw = i;
          break;
        }
      }

      if (index_yaw != -1) {
        model_->GetLink(yaw_servo_joint_.parent_link)->GetChildJoints()[index_yaw]->SetPosition(0, offset_yaw_);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera yaw joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), yaw_servo_joint_.parent_link.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: model has no link %s", ros::this_node::getName().c_str(), yaw_servo_joint_.parent_link.c_str());
  }
}
//}

/* publishCameraOrientation() //{ */
void ServoCameraPlugin::publishCameraOrientation() {
  std::scoped_lock lock(mutex_orientation_setpoint_);

  std_msgs::Float32MultiArray msg;

  std_msgs::MultiArrayLayout layout;
  layout.dim.resize(1);
  layout.dim[0].label  = "RPY";
  layout.dim[0].size   = 3;
  layout.dim[0].stride = 3;

  msg.layout = layout;
  msg.data.push_back(offset_roll_);
  msg.data.push_back(offset_pitch_);
  msg.data.push_back(offset_yaw_);

  camera_orientation_pub_.publish(msg);
}
//}

/* publishTF() //{ */
void ServoCameraPlugin::publishTF() {
  std::scoped_lock lock(mutex_model_orientation_);

  // Yaw
  geometry_msgs::TransformStamped transform_yaw;
  transform_yaw.header.stamp    = ros::Time::now();
  transform_yaw.header.frame_id = parent_frame_name_;
  transform_yaw.child_frame_id  = yaw_servo_joint_.frame_name;

  transform_yaw.transform.translation.x = yaw_servo_joint_.offset_x;
  transform_yaw.transform.translation.y = yaw_servo_joint_.offset_y;
  transform_yaw.transform.translation.z = yaw_servo_joint_.offset_z;

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0, 0, offset_yaw_);
  transform_yaw.transform.rotation = tf2::toMsg(q_yaw);

  // Roll
  geometry_msgs::TransformStamped transform_roll;
  transform_roll.header.stamp    = ros::Time::now();
  transform_roll.header.frame_id = yaw_servo_joint_.frame_name;
  transform_roll.child_frame_id  = roll_servo_joint_.frame_name;

  transform_roll.transform.translation.x = roll_servo_joint_.offset_x;
  transform_roll.transform.translation.y = roll_servo_joint_.offset_y;
  transform_roll.transform.translation.z = roll_servo_joint_.offset_z;

  tf2::Quaternion q_roll;
  q_roll.setRPY(offset_roll_, 0, 0);
  transform_roll.transform.rotation = tf2::toMsg(q_roll);

  // Pitch
  geometry_msgs::TransformStamped transform_pitch;
  transform_pitch.header.stamp    = ros::Time::now();
  transform_pitch.header.frame_id = roll_servo_joint_.frame_name;
  transform_pitch.child_frame_id  = pitch_servo_joint_.frame_name;

  transform_pitch.transform.translation.x = pitch_servo_joint_.offset_x;
  transform_pitch.transform.translation.y = pitch_servo_joint_.offset_y;
  transform_pitch.transform.translation.z = pitch_servo_joint_.offset_z;

  tf2::Quaternion q_pitch;
  q_pitch.setRPY(0, offset_pitch_, 0);
  transform_pitch.transform.rotation = tf2::toMsg(q_pitch);

  // Publish the transforms
  tf_broadcaster_.sendTransform({transform_yaw, transform_roll, transform_pitch});
  // ROS_INFO_ONCE("[%s]: TF published.", ros::this_node::getName().c_str());
}
//}

/* OnUpdate //{ */
void ServoCameraPlugin::OnUpdate() {
  // get tilts of model
  ROS_INFO_ONCE("[%s]: Updating model orientation.", ros::this_node::getName().c_str());
  std::scoped_lock lock(mutex_model_orientation_);

  ignition::math::Pose3d model_pose = model_->WorldPose();
  model_roll_                       = model_pose.Rot().Roll();
  model_pitch_                      = model_pose.Rot().Pitch();
  model_yaw_                        = model_pose.Rot().Yaw();
}
//}

GZ_REGISTER_MODEL_PLUGIN(ServoCameraPlugin)
}  // namespace gazebo
