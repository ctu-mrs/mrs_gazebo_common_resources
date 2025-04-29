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

namespace gazebo {

struct ServoJoint {
  std::string name;
  std::string parent_link;
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
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

 private:
  physics::ModelPtr model;
  ros::NodeHandle  *nh;

  ros::ServiceServer tilt_compensation_pitch_srv;
  ros::ServiceServer tilt_compensation_roll_srv;
  ros::ServiceServer tilt_compensation_yaw_srv;

  ros::ServiceServer camera_orientation_srv;
  ros::Subscriber    camera_orientation_sub;
  ros::Publisher     camera_orientation_pub;

  ros::Timer                    camera_timer;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  void desiredOrientationCallback(const std_msgs::Float32MultiArray &desired_orientation);
  bool setOrientationCallback(mrs_msgs::Vec4Request &req, mrs_msgs::Vec4Response &res);

  bool triggerTiltCompensationPitchCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool triggerTiltCompensationRollCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool triggerTiltCompensationYawCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  void   updateModelOrientation();
  void   publishTF();
  void   publishCameraOrientation();
  void   OnUpdate();
  void   cameraTimerCallback(const ros::TimerEvent &event);
  void   moveCamera();
  double getBoundedAngle(double angle);

  // camera and joint names
  std::string parent_name;

  ServoJoint roll_servo_joint;
  ServoJoint pitch_servo_joint;
  ServoJoint yaw_servo_joint;

  // camera offset with respect to model
  double offset_roll  = 0.0;
  double offset_pitch = 0.0;
  double offset_yaw   = 0.0;

  double update_rate;  // update rate of camera position

  double desired_pitch;
  double desired_roll;
  double desired_yaw;

  std::mutex mutex_desired_orientation;

  // model orientation in world frame
  std::mutex mutex_model_orientation;
  double     model_roll;
  double     model_pitch;
  double     model_yaw;

  bool initialized = false;

  event::ConnectionPtr updateConnection;
};
//}

/* Load */  //{
void ServoCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  ROS_INFO("[%s]: Servo camera plugin started.", ros::this_node::getName().c_str());
  model       = _parent;
  parent_name = model->GetName().c_str();

  /* Load params //{ */
  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->Get<double>("update_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Update_rate not defined. Setting to default value.", parent_name.c_str());
    update_rate = 30;
  }

  if (_sdf->HasElement("yaw")) {
    auto yaw_elem = _sdf->GetElement("yaw");

    if (yaw_elem->HasElement("joint_name"))
      yaw_servo_joint.name = yaw_elem->Get<std::string>("joint_name");
    else
      ROS_WARN("[%s][Servo camera]: Joint name yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("parent_link"))
      yaw_servo_joint.parent_link = yaw_elem->Get<std::string>("parent_link");
    else
      ROS_WARN("[%s][Servo camera]: Parent link yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("min_angle"))
      yaw_servo_joint.min_angle = yaw_elem->Get<double>("min_angle");
    else
      ROS_WARN("[%s][Servo camera]: Min angle yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("max_angle"))
      yaw_servo_joint.max_angle = yaw_elem->Get<double>("max_angle");
    else
      ROS_WARN("[%s][Servo camera]: Max angle yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("max_rate"))
      yaw_servo_joint.max_rate = yaw_elem->Get<double>("max_rate");
    else
      ROS_WARN("[%s][Servo camera]: Max rate yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("compensate_tilt"))
      yaw_servo_joint.compensate_tilt = yaw_elem->Get<bool>("compensate_tilt");
    else
      ROS_WARN("[%s][Servo camera]: Compensate tilt yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("offset_x"))
      yaw_servo_joint.offset_x = yaw_elem->Get<double>("offset_x");
    else
      ROS_WARN("[%s][Servo camera]: Offset x yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("offset_y"))
      yaw_servo_joint.offset_y = yaw_elem->Get<double>("offset_y");
    else
      ROS_WARN("[%s][Servo camera]: Offset y yaw not defined.", parent_name.c_str());

    if (yaw_elem->HasElement("offset_z"))
      yaw_servo_joint.offset_z = yaw_elem->Get<double>("offset_z");
    else
      ROS_WARN("[%s][Servo camera]: Offset z yaw not defined.", parent_name.c_str());

  } else {
    ROS_WARN("[%s][Servo camera]: Yaw joint not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("roll")) {
    auto roll_elem = _sdf->GetElement("roll");
    if (roll_elem->HasElement("joint_name"))
      roll_servo_joint.name = roll_elem->Get<std::string>("joint_name");
    else
      ROS_WARN("[%s][Servo camera]: Joint name roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("parent_link"))
      roll_servo_joint.parent_link = roll_elem->Get<std::string>("parent_link");
    else
      ROS_WARN("[%s][Servo camera]: Parent link roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("min_angle"))
      roll_servo_joint.min_angle = roll_elem->Get<double>("min_angle");
    else
      ROS_WARN("[%s][Servo camera]: Min angle roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("max_angle"))
      roll_servo_joint.max_angle = roll_elem->Get<double>("max_angle");
    else
      ROS_WARN("[%s][Servo camera]: Max angle roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("max_rate"))
      roll_servo_joint.max_rate = roll_elem->Get<double>("max_rate");
    else
      ROS_WARN("[%s][Servo camera]: Max rate roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("compensate_tilt"))
      roll_servo_joint.compensate_tilt = roll_elem->Get<bool>("compensate_tilt");
    else
      ROS_WARN("[%s][Servo camera]: Compensate tilt roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("offset_x"))
      roll_servo_joint.offset_x = roll_elem->Get<double>("offset_x");
    else
      ROS_WARN("[%s][Servo camera]: Offset x roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("offset_y"))
      roll_servo_joint.offset_y = roll_elem->Get<double>("offset_y");
    else
      ROS_WARN("[%s][Servo camera]: Offset y roll not defined.", parent_name.c_str());

    if (roll_elem->HasElement("offset_z"))
      roll_servo_joint.offset_z = roll_elem->Get<double>("offset_z");
    else
      ROS_WARN("[%s][Servo camera]: Offset z roll not defined.", parent_name.c_str());
  } else {
    ROS_WARN("[%s][Servo camera]: Roll joint not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("pitch")) {
    auto pitch_elem = _sdf->GetElement("pitch");
    if (pitch_elem->HasElement("joint_name"))
      pitch_servo_joint.name = pitch_elem->Get<std::string>("joint_name");
    else
      ROS_WARN("[%s][Servo camera]: Joint name pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("parent_link"))
      pitch_servo_joint.parent_link = pitch_elem->Get<std::string>("parent_link");
    else
      ROS_WARN("[%s][Servo camera]: Parent link pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("min_angle"))
      pitch_servo_joint.min_angle = pitch_elem->Get<double>("min_angle");
    else
      ROS_WARN("[%s][Servo camera]: Min angle pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("max_angle"))
      pitch_servo_joint.max_angle = pitch_elem->Get<double>("max_angle");
    else
      ROS_WARN("[%s][Servo camera]: Max angle pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("max_rate"))
      pitch_servo_joint.max_rate = pitch_elem->Get<double>("max_rate");
    else
      ROS_WARN("[%s][Servo camera]: Max rate pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("compensate_tilt"))
      pitch_servo_joint.compensate_tilt = pitch_elem->Get<bool>("compensate_tilt");
    else
      ROS_WARN("[%s][Servo camera]: Compensate tilt pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("offset_x"))
      pitch_servo_joint.offset_x = pitch_elem->Get<double>("offset_x");
    else
      ROS_WARN("[%s][Servo camera]: Offset x pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("offset_y"))
      pitch_servo_joint.offset_y = pitch_elem->Get<double>("offset_y");
    else
      ROS_WARN("[%s][Servo camera]: Offset y pitch not defined.", parent_name.c_str());

    if (pitch_elem->HasElement("offset_z"))
      pitch_servo_joint.offset_z = pitch_elem->Get<double>("offset_z");
    else
      ROS_WARN("[%s][Servo camera]: Offset z pitch not defined.", parent_name.c_str());
  } else {
    ROS_WARN("[%s][Servo camera]: Pitch joint not defined.", parent_name.c_str());
  }
  //}

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "servo_camera_plugin", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle("~");

  // ROS subscriber for control of roll and pitch of camera
  std::stringstream ss;
  ss << "/" << parent_name << "/servo_camera/desired_orientation";
  camera_orientation_sub = nh->subscribe(ss.str().c_str(), 1, &ServoCameraPlugin::desiredOrientationCallback, this);

  // ROS service to control the camera orientation
  ss.str(std::string());
  ss << "/" << parent_name << "/servo_camera/set_orientation";
  camera_orientation_srv = nh->advertiseService(ss.str().c_str(), &ServoCameraPlugin::setOrientationCallback, this);

  // ROS publisher for camera orientation
  ss.str(std::string());
  ss << "/" << parent_name << "/servo_camera/camera_orientation";
  camera_orientation_pub = nh->advertise<std_msgs::Float32MultiArray>(ss.str().c_str(), 1);

  // ROS services for triggering roll and pitch tilt compensation
  ss.str(std::string());
  ss << "/" << parent_name << "/servo_camera/compensate_tilt_pitch";
  tilt_compensation_pitch_srv = nh->advertiseService(ss.str().c_str(), &ServoCameraPlugin::triggerTiltCompensationPitchCallback, this);

  ss.str(std::string());
  ss << "/" << parent_name << "/servo_camera/compensate_tilt_roll";
  tilt_compensation_roll_srv = nh->advertiseService(ss.str().c_str(), &ServoCameraPlugin::triggerTiltCompensationRollCallback, this);

  ss.str(std::string());
  ss << "/" << parent_name << "/servo_camera/compensate_tilt_yaw";
  tilt_compensation_yaw_srv = nh->advertiseService(ss.str().c_str(), &ServoCameraPlugin::triggerTiltCompensationYawCallback, this);

  camera_timer = nh->createTimer(ros::Rate(update_rate), &ServoCameraPlugin::cameraTimerCallback, this);

  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ServoCameraPlugin::OnUpdate, this));

  initialized = true;

  ROS_INFO("[%s][Servo camera]: Servo camera initialized.", parent_name.c_str());
}
//}

/* cameraTimerCallback */  //{
void ServoCameraPlugin::cameraTimerCallback(const ros::TimerEvent &event) {
  if (!initialized) return;

  moveCamera();
  publishTF();
}
//}

/* desiredOrientationCallback */  //{
void ServoCameraPlugin::desiredOrientationCallback(const std_msgs::Float32MultiArray &msg) {
  if (msg.data.size() > 3 || msg.data.size() < 1) {
    ROS_INFO("[%s][Servo camera]: cameraOrientationCallback: Invalid request. Unexpected size of data.", parent_name.c_str());
    return;
  }

  std::scoped_lock lock(mutex_desired_orientation);

  desired_roll = fmin(fmax(roll_servo_joint.min_angle, msg.data[0]), roll_servo_joint.max_angle);  // clip value in limits
  desired_roll = getBoundedAngle(desired_roll);
  ROS_INFO("[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name.c_str(), desired_pitch, msg.data[0]);
  if (msg.data.size() == 1) return;

  desired_pitch = fmin(fmax(pitch_servo_joint.min_angle, msg.data[1]), pitch_servo_joint.max_angle);  // clip value in limits
  desired_pitch = getBoundedAngle(desired_pitch);
  ROS_INFO("[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name.c_str(), desired_pitch, msg.data[1]);
  if (msg.data.size() == 2) return;

  desired_yaw = fmin(fmax(yaw_servo_joint.min_angle, msg.data[2]), yaw_servo_joint.max_angle);  // clip value in limits
  desired_yaw = getBoundedAngle(desired_yaw);
  ROS_INFO("[%s][Servo camera]: Change camera yaw to %.2f! Requested unlimited yaw = %.2f", parent_name.c_str(), desired_yaw, msg.data[2]);
  ROS_INFO("[%s][Servo camera]: ---", parent_name.c_str());
}
//}

/* setOrientationCallback() //{ */
bool ServoCameraPlugin::setOrientationCallback(mrs_msgs::Vec4Request &req, mrs_msgs::Vec4Response &res) {
  if (req.goal[3] != 0) {
    res.success = false;
    res.message = "Camera orientation message received with non-zero value in 4th element.";

    ROS_WARN("[%s][Servo camera]: %s", parent_name.c_str(), res.message.c_str());
    return true;
  }

  std::scoped_lock lock(mutex_desired_orientation);

  double new_desired_roll = fmin(fmax(roll_servo_joint.min_angle, req.goal[0]), roll_servo_joint.max_angle);  // clip value in limits
  new_desired_roll        = getBoundedAngle(new_desired_roll);
  ROS_INFO("[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name.c_str(), new_desired_roll, req.goal[0]);

  double new_desired_pitch = fmin(fmax(pitch_servo_joint.min_angle, req.goal[1]), pitch_servo_joint.max_angle);  // clip value in limits
  new_desired_pitch        = getBoundedAngle(new_desired_pitch);
  ROS_INFO("[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name.c_str(), new_desired_pitch, req.goal[1]);

  double new_desired_yaw = fmin(fmax(yaw_servo_joint.min_angle, req.goal[2]), yaw_servo_joint.max_angle);  // clip value in limits
  new_desired_yaw        = getBoundedAngle(new_desired_yaw);
  ROS_INFO("[%s][Servo camera]: Change camera yaw to %.2f! Requested unlimited yaw = %.2f", parent_name.c_str(), new_desired_yaw, req.goal[2]);

  desired_roll  = new_desired_roll;
  desired_pitch = new_desired_pitch;
  desired_yaw   = new_desired_yaw;

  std::stringstream ss;
  ss << "Camera orientation processed. " << std::fixed << std::setprecision(2) 
     << "Roll: requested = " << req.goal[0] << ", limited = " << desired_roll << "; "
     << "Pitch: requested = " << req.goal[1] << ", limited = " << desired_pitch << "; "
     << "Yaw: requested = " << req.goal[2] << ", limited = " << desired_yaw << ".";

  res.success = true;
  res.message = ss.str();

  return true;
}
//}

/* triggerTiltCompensationPitchCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationPitchCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation pitch!", parent_name.c_str());

  pitch_servo_joint.compensate_tilt = req.data;
  res.message                       = pitch_servo_joint.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* triggerTiltCompensationRollCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationRollCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation roll!", parent_name.c_str());

  roll_servo_joint.compensate_tilt = req.data;
  res.message                      = roll_servo_joint.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* triggerTiltCompensationYawCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationYawCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation yaw!", parent_name.c_str());

  yaw_servo_joint.compensate_tilt = req.data;
  res.message                     = yaw_servo_joint.compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* getBoundedAngle(double angle) //{ */
double ServoCameraPlugin::getBoundedAngle(double angle) { return angle > M_PI ? -2 * M_PI + angle : angle < -M_PI ? 2 * M_PI + angle : angle; }
//}

/* moveCamera() //{ */
void ServoCameraPlugin::moveCamera() {
  {
    std::scoped_lock lock(mutex_desired_orientation, mutex_model_orientation);

    // get desired pitch and roll angles compensated for current tilts of model
    double compensated_pitch = pitch_servo_joint.compensate_tilt ? desired_pitch - model_pitch : desired_pitch;
    double compensated_roll  = roll_servo_joint.compensate_tilt ? desired_roll - model_roll : desired_roll;
    double compensated_yaw   = yaw_servo_joint.compensate_tilt ? desired_yaw - model_yaw : desired_yaw;

    // find required angle in next step that satisfies limits on angular_rates
    double abs_diff_pitch = abs(offset_pitch - compensated_pitch);
    double min_diff_pitch = abs_diff_pitch > M_PI ? 2 * M_PI - abs_diff_pitch : abs_diff_pitch;

    if (abs(min_diff_pitch) > 1e-4) {
      double dir = compensated_pitch > offset_pitch ? 1.0 : -1.0;
      dir        = abs(min_diff_pitch - abs_diff_pitch) > 1e-4 ? -1.0 * dir : dir;
      offset_pitch += dir * fmin(abs(min_diff_pitch), pitch_servo_joint.max_rate / update_rate);
      offset_pitch = getBoundedAngle(offset_pitch);
    }

    double abs_diff_roll = abs(offset_roll - compensated_roll);
    double min_diff_roll = abs_diff_roll > M_PI ? 2 * M_PI - abs_diff_roll : abs_diff_roll;

    if (abs(min_diff_roll) > 1e-4) {
      double dir = compensated_roll > offset_roll ? 1.0 : -1.0;
      dir        = abs(min_diff_roll - abs_diff_roll) > 1e-4 ? -1.0 * dir : dir;
      offset_roll += dir * fmin(abs(min_diff_roll), roll_servo_joint.max_rate / update_rate);
      offset_roll = getBoundedAngle(offset_roll);
    }

    double abs_diff_yaw = abs(offset_yaw - compensated_yaw);
    double min_diff_yaw = abs_diff_yaw > M_PI ? 2 * M_PI - abs_diff_yaw : abs_diff_yaw;

    if (abs(min_diff_yaw) > 1e-4) {
      double dir = compensated_yaw > offset_yaw ? 1.0 : -1.0;
      dir        = abs(min_diff_yaw - abs_diff_yaw) > 1e-4 ? -1.0 * dir : dir;
      offset_yaw += dir * fmin(abs(min_diff_yaw), yaw_servo_joint.max_rate / update_rate);
      offset_yaw = getBoundedAngle(offset_yaw);
    }
  }

  // set joint coordinates based on computed required orientations of links
  if (model->GetLink(roll_servo_joint.name) != NULL) {
    if (model->GetLink(roll_servo_joint.parent_link)->GetChildJoints().size() != 0) {
      int index_roll = -1;
      for (size_t i = 0; i < model->GetLink(roll_servo_joint.parent_link)->GetChildJoints().size(); i++) {
        if (model->GetLink(roll_servo_joint.parent_link)->GetChildJoints()[i]->GetName() == roll_servo_joint.name) {
          index_roll = i;
          break;
        }
      }

      if (index_roll != -1) {
        model->GetLink(roll_servo_joint.parent_link)->GetChildJoints()[index_roll]->SetPosition(0, offset_roll);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera roll joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), roll_servo_joint.parent_link.c_str());
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no link %s", ros::this_node::getName().c_str(), roll_servo_joint.parent_link.c_str());
  }

  if (model->GetLink(pitch_servo_joint.parent_link) != NULL) {
    if (model->GetLink(pitch_servo_joint.parent_link)->GetChildJoints().size() != 0) {
      int index_pitch = -1;
      for (size_t i = 0; i < model->GetLink(pitch_servo_joint.parent_link)->GetChildJoints().size(); i++) {
        if (model->GetLink(pitch_servo_joint.parent_link)->GetChildJoints()[i]->GetName() == pitch_servo_joint.name) {
          index_pitch = i;
          break;
        }
      }

      if (index_pitch != -1) {
        model->GetLink(pitch_servo_joint.parent_link)->GetChildJoints()[index_pitch]->SetPosition(0, offset_pitch);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera pitch joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), pitch_servo_joint.parent_link.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no link %s", ros::this_node::getName().c_str(), pitch_servo_joint.parent_link.c_str());
  }

  if (model->GetLink(yaw_servo_joint.parent_link) != NULL) {
    if (model->GetLink(yaw_servo_joint.parent_link)->GetChildJoints().size() != 0) {
      int index_yaw = -1;
      for (size_t i = 0; i < model->GetLink(yaw_servo_joint.parent_link)->GetChildJoints().size(); i++) {
        if (model->GetLink(yaw_servo_joint.parent_link)->GetChildJoints()[i]->GetName() == yaw_servo_joint.name) {
          index_yaw = i;
          break;
        }
      }

      if (index_yaw != -1) {
        model->GetLink(yaw_servo_joint.parent_link)->GetChildJoints()[index_yaw]->SetPosition(0, offset_yaw);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera yaw joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), yaw_servo_joint.parent_link.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no link %s", ros::this_node::getName().c_str(), yaw_servo_joint.parent_link.c_str());
  }
}
//}

/* publishCameraOrientation() //{ */
void ServoCameraPlugin::publishCameraOrientation() {
  std::scoped_lock lock(mutex_desired_orientation);

  std_msgs::Float32MultiArray msg;

  std_msgs::MultiArrayLayout layout;
  layout.dim.resize(1);
  layout.dim[0].label  = "RPY";
  layout.dim[0].size   = 3;
  layout.dim[0].stride = 3;

  msg.layout = layout;
  msg.data.push_back(offset_roll);
  msg.data.push_back(offset_pitch);
  msg.data.push_back(offset_yaw);

  camera_orientation_pub.publish(msg);
}
//}

/* updateCameraPosition() //{ */
void ServoCameraPlugin::updateModelOrientation() {
  // get tilts of model
  ROS_INFO_ONCE("[%s]: Updating model orientation.", ros::this_node::getName().c_str());
  std::scoped_lock lock(mutex_model_orientation);

  ignition::math::Pose3d model_pose = model->WorldPose();
  model_roll                        = model_pose.Rot().Roll();
  model_pitch                       = model_pose.Rot().Pitch();
  model_yaw                         = model_pose.Rot().Yaw();
}
//}

/* publishTF() //{ */
void ServoCameraPlugin::publishTF() {
  std::scoped_lock lock(mutex_model_orientation);

  tf2::Quaternion q;
  q.setRPY(offset_roll, offset_pitch, offset_yaw);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp    = ros::Time::now();
  transform.header.frame_id = "uav1/fcu";
  transform.child_frame_id  = "uav1/servo_camera";

  transform.transform.translation.x = 0;
  transform.transform.translation.y = 0;
  transform.transform.translation.z = 0;

  transform.transform.rotation = tf2::toMsg(q);

  // Send the transform
  tf_broadcaster_.sendTransform(transform);
  // ROS_INFO_ONCE("[%s]: TF published.", ros::this_node::getName().c_str());
}
//}

/* OnUpdate //{ */
void ServoCameraPlugin::OnUpdate() { updateModelOrientation(); }
//}

GZ_REGISTER_MODEL_PLUGIN(ServoCameraPlugin)
}  // namespace gazebo
