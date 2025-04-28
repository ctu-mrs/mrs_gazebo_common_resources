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

/* Class definition */  //{
class ServoCameraPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

private:
  physics::ModelPtr  model;
  ros::NodeHandle *  nh;
  
  ros::ServiceServer tilt_compensation_pitch_srv;
  ros::ServiceServer tilt_compensation_roll_srv;
  ros::ServiceServer tilt_compensation_yaw_srv;

  ros::ServiceServer camera_orientation_srv;
  ros::Subscriber    camera_orientation_sub;
  ros::Publisher     camera_orientation_pub;

  ros::Timer         camera_timer;
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

  // camera and gimbal link and joint names
  std::string parent_name;

  std::string joint_name_pitch;
  std::string joint_name_roll;
  std::string joint_name_yaw;

  std::string parent_link_pitch;
  std::string parent_link_roll;
  std::string parent_link_yaw;

  // camera offset with respect to model
  double offset_roll  = 0.0;
  double offset_pitch = 0.0;
  double offset_yaw   = 0.0;

  double update_rate;  // update rate of camera position

  double desired_pitch;
  double min_pitch;
  double max_pitch;
  double max_pitch_rate;
  bool   compensate_tilt_pitch;

  double desired_roll;
  double min_roll;
  double max_roll;
  double max_roll_rate;
  bool   compensate_tilt_roll;

  double desired_yaw;
  double min_yaw;
  double max_yaw;
  double max_yaw_rate;
  bool   compensate_tilt_yaw;

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

  if (_sdf->HasElement("max_pitch_rate")) {
    max_pitch_rate = _sdf->Get<double>("max_pitch_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Max pitch rate not defined. Setting to default value.", parent_name.c_str());
    max_pitch_rate = 10;
  }

  if (_sdf->HasElement("max_roll_rate")) {
    max_roll_rate = _sdf->Get<double>("max_roll_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Max roll rate not defined. Setting to default value.", parent_name.c_str());
    max_roll_rate = 10;
  }

  if (_sdf->HasElement("max_yaw_rate")) {
    max_yaw_rate = _sdf->Get<double>("max_yaw_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Max yaw rate not defined. Setting to default value.", parent_name.c_str());
    max_yaw_rate = 10;
  }

  if (_sdf->HasElement("min_pitch")) {
    min_pitch = _sdf->Get<double>("min_pitch");
  } else {
    ROS_WARN("[%s][Servo camera]: min_pitch not defined. Setting to default value.", parent_name.c_str());
    min_pitch = -1.57;
  }

  if (_sdf->HasElement("max_pitch")) {
    max_pitch = _sdf->Get<double>("max_pitch");
  } else {
    ROS_WARN("[%s][Servo camera]: max_pitch not defined. Setting to default value.", parent_name.c_str());
    max_pitch = 1.57;
  }

  if (_sdf->HasElement("min_roll")) {
    min_roll = _sdf->Get<double>("min_roll");
  } else {
    ROS_WARN("[%s][Servo camera]: min_roll not defined. Setting to default value.", parent_name.c_str());
    min_roll = -0.5;
  }

  if (_sdf->HasElement("max_roll")) {
    max_roll = _sdf->Get<double>("max_roll");
  } else {
    ROS_WARN("[%s][Servo camera]: max_roll not defined. Setting to default value.", parent_name.c_str());
    max_roll = 0.5;
  }

  if (_sdf->HasElement("min_yaw")) {
    min_yaw = _sdf->Get<double>("min_yaw");
  } else {
    ROS_WARN("[%s][Servo camera]: min_yaw not defined. Setting to default value.", parent_name.c_str());
    min_yaw = -1.57;
  }

  if (_sdf->HasElement("max_yaw")) {
    max_yaw = _sdf->Get<double>("max_yaw");
  } else {
    ROS_WARN("[%s][Servo camera]: max_yaw not defined. Setting to default value.", parent_name.c_str());
    max_yaw = 1.57;
  }

  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->Get<double>("update_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Update_rate not defined. Setting to default value.", parent_name.c_str());
    update_rate = 30;
  }

  if (_sdf->HasElement("joint_name_pitch")) {
    joint_name_pitch = _sdf->Get<std::string>("joint_name_pitch");
  } else {
    ROS_WARN("[%s][Servo camera]: Joint name pitch not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("joint_name_roll")) {
    joint_name_roll = _sdf->Get<std::string>("joint_name_roll");
  } else {
    ROS_WARN("[%s][Servo camera]: Joint name roll not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("joint_name_yaw")) {
    joint_name_yaw = _sdf->Get<std::string>("joint_name_yaw");
  } else {
    ROS_WARN("[%s][Servo camera]: Joint name yaw not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("parent_link_pitch")) {
    parent_link_pitch = _sdf->Get<std::string>("parent_link_pitch");
  } else {
    ROS_WARN("[%s][Servo camera]: Parent link pitch not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("parent_link_roll")) {
    parent_link_roll = _sdf->Get<std::string>("parent_link_roll");
  } else {
    ROS_WARN("[%s][Servo camera]: Parent link roll not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("parent_link_yaw")) {
    parent_link_yaw = _sdf->Get<std::string>("parent_link_yaw");
  } else {
    ROS_WARN("[%s][Servo camera]: Parent link yaw not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("compensate_tilt_pitch")) {
    compensate_tilt_pitch = _sdf->Get<bool>("compensate_tilt_pitch");
  } else {
    ROS_WARN("[%s][Servo camera]: Tilt pitch compensation not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("compensate_tilt_roll")) {
    compensate_tilt_roll = _sdf->Get<bool>("compensate_tilt_roll");
  } else {
    ROS_WARN("[%s][Servo camera]: Tilt roll compensation not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("compensate_tilt_yaw")) {
    compensate_tilt_yaw = _sdf->Get<bool>("compensate_tilt_yaw");
  } else {
    ROS_WARN("[%s][Servo camera]: Tilt yaw compensation not defined.", parent_name.c_str());
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

  if (!initialized) {
    return;
  }

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
  
  desired_roll = fmin(fmax(min_roll, msg.data[0]), max_roll);  // clip value in limits
  desired_roll = getBoundedAngle(desired_roll);
  ROS_INFO("[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name.c_str(), desired_pitch, msg.data[0]);
  if (msg.data.size() == 1) return;
  
  desired_pitch = fmin(fmax(min_pitch, msg.data[1]), max_pitch);  // clip value in limits
  desired_pitch = getBoundedAngle(desired_pitch);
  ROS_INFO("[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name.c_str(), desired_pitch, msg.data[1]);
  if (msg.data.size() == 2) return;

  desired_yaw = fmin(fmax(min_yaw, msg.data[2]), max_yaw);  // clip value in limits
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

    ROS_WARN("[%s][Servo camera]: %s", parent_name.c_str(),res.message.c_str());
    return true;
  }

  std::scoped_lock lock(mutex_desired_orientation);

  double new_desired_roll = fmin(fmax(min_roll, req.goal[0]), max_roll);  // clip value in limits
  new_desired_roll = getBoundedAngle(new_desired_roll);
  ROS_INFO("[%s][Servo camera]: Change camera roll to %.2f! Requested unlimited roll = %.2f", parent_name.c_str(), new_desired_roll, req.goal[0]);

  double new_desired_pitch = fmin(fmax(min_pitch, req.goal[1]), max_pitch);  // clip value in limits
  new_desired_pitch = getBoundedAngle(new_desired_pitch);
  ROS_INFO("[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name.c_str(), new_desired_pitch, req.goal[1]);

  double new_desired_yaw = fmin(fmax(min_yaw, req.goal[2]), max_yaw);  // clip value in limits
  new_desired_yaw = getBoundedAngle(new_desired_yaw);
  ROS_INFO("[%s][Servo camera]: Change camera yaw to %.2f! Requested unlimited yaw = %.2f", parent_name.c_str(), new_desired_yaw, req.goal[2]);

  desired_roll = new_desired_roll;
  desired_pitch = new_desired_pitch;
  desired_yaw = new_desired_yaw;

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

  compensate_tilt_pitch = req.data;
  res.message           = compensate_tilt_pitch ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* triggerTiltCompensationRollCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationRollCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {

  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation roll!", parent_name.c_str());

  compensate_tilt_roll = req.data;
  res.message          = compensate_tilt_roll ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name.c_str(), res.message.c_str());

  res.success = true;
  return res.success;
}
//}

/* triggerTiltCompensationYawCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationYawCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {

  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation yaw!", parent_name.c_str());

  compensate_tilt_yaw = req.data;
  res.message         = compensate_tilt_yaw ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: %s.", parent_name.c_str(), res.message.c_str());

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
    std::scoped_lock lock(mutex_desired_orientation, mutex_model_orientation);

    // get desired pitch and roll angles compensated for current tilts of model
    double compensated_pitch = compensate_tilt_pitch ? desired_pitch - model_pitch : desired_pitch;
    double compensated_roll  = compensate_tilt_roll ? desired_roll - model_roll : desired_roll;
    double compensated_yaw   = compensate_tilt_yaw ? desired_yaw - model_yaw : desired_yaw;

    // find required angle in next step that satisfies limits on angular_rates
    double abs_diff_pitch = abs(offset_pitch - compensated_pitch);
    double min_diff_pitch = abs_diff_pitch > M_PI ? 2 * M_PI - abs_diff_pitch : abs_diff_pitch;

    if (abs(min_diff_pitch) > 1e-4) {

      double dir = compensated_pitch > offset_pitch ? 1.0 : -1.0;
      dir        = abs(min_diff_pitch - abs_diff_pitch) > 1e-4 ? -1.0 * dir : dir;
      offset_pitch += dir * fmin(abs(min_diff_pitch), max_pitch_rate / update_rate);
      offset_pitch = getBoundedAngle(offset_pitch);
    }

    double abs_diff_roll = abs(offset_roll - compensated_roll);
    double min_diff_roll = abs_diff_roll > M_PI ? 2 * M_PI - abs_diff_roll : abs_diff_roll;

    if (abs(min_diff_roll) > 1e-4) {

      double dir = compensated_roll > offset_roll ? 1.0 : -1.0;
      dir        = abs(min_diff_roll - abs_diff_roll) > 1e-4 ? -1.0 * dir : dir;
      offset_roll += dir * fmin(abs(min_diff_roll), max_roll_rate / update_rate);
      offset_roll = getBoundedAngle(offset_roll);
    }

    double abs_diff_yaw = abs(offset_yaw - compensated_yaw);
    double min_diff_yaw = abs_diff_yaw > M_PI ? 2 * M_PI - abs_diff_yaw : abs_diff_yaw;

    if (abs(min_diff_yaw) > 1e-4) {

      double dir = compensated_yaw > offset_yaw ? 1.0 : -1.0;
      dir        = abs(min_diff_yaw - abs_diff_yaw) > 1e-4 ? -1.0 * dir : dir;
      offset_yaw += dir * fmin(abs(min_diff_yaw), max_yaw_rate / update_rate);
      offset_yaw = getBoundedAngle(offset_yaw);
    }
  }

  // set joint coordinates based on computed required orientations of links
  if (model->GetLink(parent_link_roll) != NULL) {

    if (model->GetLink(parent_link_roll)->GetChildJoints().size() != 0) {

      int index_roll = -1;
      for (size_t i = 0; i < model->GetLink(parent_link_roll)->GetChildJoints().size(); i++) {

        if (model->GetLink(parent_link_roll)->GetChildJoints()[i]->GetName() == joint_name_roll) {
          index_roll = i;
          break;
        }
      }

      if (index_roll != -1) {
        model->GetLink(parent_link_roll)->GetChildJoints()[index_roll]->SetPosition(0, offset_roll);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera roll joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), parent_link_roll.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no link %s", ros::this_node::getName().c_str(), parent_link_roll.c_str());
  }

  if (model->GetLink(parent_link_pitch) != NULL) {

    if (model->GetLink(parent_link_pitch)->GetChildJoints().size() != 0) {

      int index_pitch = -1;
      for (size_t i = 0; i < model->GetLink(parent_link_pitch)->GetChildJoints().size(); i++) {

        if (model->GetLink(parent_link_pitch)->GetChildJoints()[i]->GetName() == joint_name_pitch) {
          index_pitch = i;
          break;
        }
      }

      if (index_pitch != -1) {
        model->GetLink(parent_link_pitch)->GetChildJoints()[index_pitch]->SetPosition(0, offset_pitch);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera pitch joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), parent_link_pitch.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no link %s", ros::this_node::getName().c_str(), parent_link_pitch.c_str());
  }

  if (model->GetLink(parent_link_yaw) != NULL) {

    if (model->GetLink(parent_link_yaw)->GetChildJoints().size() != 0) {

      int index_yaw = -1;
      for (size_t i = 0; i < model->GetLink(parent_link_yaw)->GetChildJoints().size(); i++) {

        if (model->GetLink(parent_link_yaw)->GetChildJoints()[i]->GetName() == joint_name_yaw) {
          index_yaw = i;
          break;
        }
      }

      if (index_yaw != -1) {
        model->GetLink(parent_link_yaw)->GetChildJoints()[index_yaw]->SetPosition(0, offset_yaw);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera yaw joint did not found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Link %s has no child joints", ros::this_node::getName().c_str(), parent_link_yaw.c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no link %s", ros::this_node::getName().c_str(), parent_link_yaw.c_str());
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
  std::scoped_lock       lock(mutex_model_orientation);
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
void ServoCameraPlugin::OnUpdate() {
  updateModelOrientation();
}
//}

GZ_REGISTER_MODEL_PLUGIN(ServoCameraPlugin)
}  // namespace gazebo
