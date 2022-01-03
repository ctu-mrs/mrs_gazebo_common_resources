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
#include <std_msgs/Float32.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

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
  ros::ServiceServer tilt_compensation_srv;
  ros::ServiceClient spawn_srv, change_state_srv;
  ros::Subscriber    pitch_sub;
  ros::Timer         camera_timer;

  void pitchCallback(const std_msgs::Float32 &desired_pitch);
  bool triggerTiltCompensationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  void updateCameraPosition();
  void OnUpdate();
  void cameraTimerCallback(const ros::TimerEvent &event);
  void moveCamera();
  void convertEulerToQuaternion(double roll_offset, double pitch_offset, double yaw_offset);

  std::string            camera_model;
  std::string            parent_name;
  ignition::math::Pose3d offset;
  ignition::math::Pose3d spawn_point;
  std::string            joint_name;
  std::string            camera_type;
  double                 offset_roll;
  double                 offset_pitch;
  double                 offset_yaw;
  double                 update_rate;
  double                 max_pitch_rate;
  double                 min_pitch;
  double                 max_pitch;
  double                 desired_pitch;
  double                 spawn_delay_sec = 5.0;
  int                    spawn_delay_remaining_loops;
  bool                   camera_spawned;
  bool                   compensate_tilt;
  std::mutex             mutex_desired_pitch;

  bool initialized = false;

  event::ConnectionPtr updateConnection;
};
//}

/* Load */  //{
void ServoCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  ROS_INFO("[%s]: Servo camera plugin started.", ros::this_node::getName().c_str());
  model        = _parent;
  parent_name  = model->GetName().c_str();
  offset_yaw   = 0.0;
  offset_pitch = 0.0;
  offset_roll  = 0.0;

/* Load params //{ */

  if (_sdf->HasElement("max_pitch_rate")) {
    max_pitch_rate = _sdf->Get<double>("max_pitch_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: update_rate not defined. Setting to default value.", parent_name.c_str());
    max_pitch_rate = 10;
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

  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->Get<double>("update_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Update_rate not defined. Setting to default value.", parent_name.c_str());
    update_rate = 30;
  }

  if (_sdf->HasElement("joint_name")) {
    joint_name = _sdf->Get<std::string>("joint_name");
  } else {
    ROS_WARN("[%s][Servo camera]: Joint name not defined.", parent_name.c_str());
  }

  if (_sdf->HasElement("compensate_tilt")) {
    compensate_tilt = _sdf->Get<bool>("compensate_tilt");
  } else {
    ROS_WARN("[%s][Servo camera]: Tilt compensation not defined.", parent_name.c_str());
  }

//}

  convertEulerToQuaternion(offset_roll, offset_pitch, offset_yaw);

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "servo_camera_plugin", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle("~");

  change_state_srv = nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  spawn_srv        = nh->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

  // create ROS services to control the light
  std::stringstream ss;
  ss << "/" << parent_name << "/servo_camera/set_pitch";
  pitch_sub = nh->subscribe(ss.str().c_str(), 1, &ServoCameraPlugin::pitchCallback, this);
  ss.str(std::string());
  ss << "/" << parent_name << "/servo_camera/compensate_tilt";
  tilt_compensation_srv = nh->advertiseService(ss.str().c_str(), &ServoCameraPlugin::triggerTiltCompensationCallback, this);

  camera_timer = nh->createTimer(ros::Rate(update_rate), &ServoCameraPlugin::cameraTimerCallback, this);

  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ServoCameraPlugin::OnUpdate, this));

  initialized              = true;

  ROS_INFO("[%s][Servo camera]: Servo camera initialized.", parent_name.c_str());
}
//}

/* convertEulerToQuaternion() //{ */
void ServoCameraPlugin::convertEulerToQuaternion(double roll_offset, double pitch_offset, double yaw_offset) {
  offset.Rot() = ignition::math::Quaternion<double>(roll_offset, pitch_offset, yaw_offset);
}
//}

/* cameraTimerCallback */  //{
void ServoCameraPlugin::cameraTimerCallback(const ros::TimerEvent &event) {

  if (!initialized) { 
    return;
  }

  moveCamera();
}
//}

/* pitchCallback */  //{
void ServoCameraPlugin::pitchCallback(const std_msgs::Float32 &msg) {

  std::scoped_lock lock(mutex_desired_pitch);
  desired_pitch = fmin(fmax(min_pitch, msg.data), max_pitch); // clip value in limits
  desired_pitch = fmod(desired_pitch + 2 * M_PI, 2 * M_PI);
  ROS_INFO("[%s][Servo camera]: Change camera pitch to %.2f! Requested unlimited pitch = %.2f", parent_name.c_str(), desired_pitch, msg.data);

}
//}

/* triggerTiltCompensationCallback() */  //{
bool ServoCameraPlugin::triggerTiltCompensationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {

  ROS_INFO("[%s][Servo camera]: Trigger tilt compensation!", parent_name.c_str());

  compensate_tilt = req.data;
  res.message     = compensate_tilt ? "tilt compensation enabled" : "tilt compensation disabled";
  ROS_WARN("[%s][Servo Camera]: Tilt compensation is not enabled in current implementation.", parent_name.c_str());

  res.success = true;
  return res.success;

}
//}

/* moveCamera() //{ */
void ServoCameraPlugin::moveCamera() {

  {
    std::scoped_lock lock(mutex_desired_pitch);

    double           abs_diff = abs(offset_pitch - desired_pitch);
    double           min_diff = abs_diff > M_PI ? 2 * M_PI - abs_diff : abs_diff;

    if (min_diff > 1e-5) {
      double dir = desired_pitch > offset_pitch ? 1.0 : -1.0;
      dir        = abs(min_diff - abs_diff) > 1e-5 ? -1.0 * dir : dir;
      offset_pitch += dir * fmin(abs(desired_pitch - offset_pitch), max_pitch_rate / update_rate);
      convertEulerToQuaternion(offset_roll, offset_pitch, offset_yaw);
      offset_pitch = fmod(offset_pitch + 2 * M_PI, 2 * M_PI);
    }

  }

  if (model->GetLink("base_link") != NULL) {

    if (model->GetLink("base_link")->GetChildJoints().size() != 0) {

      int index = -1;
      for (size_t i = 0; i < model->GetLink("base_link")->GetChildJoints().size(); i++) {
        if (model->GetLink("base_link")->GetChildJoints()[i]->GetName() == joint_name) {
          index = i;
          break;
        }
      }

      if (index != -1) {
        double applied_pitch_offset = offset_pitch > M_PI ? -2*M_PI + offset_pitch : offset_pitch;
        model->GetLink("base_link")->GetChildJoints()[index]->SetPosition(0, applied_pitch_offset);
        ROS_INFO_THROTTLE(5.0, "[%s]: Setting position of joint %s to %.2f.", ros::this_node::getName().c_str(), model->GetLink("base_link")->GetChildJoints()[index]->GetName().c_str(), applied_pitch_offset);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera macro joint doesn't found.", ros::this_node::getName().c_str());
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Base link has no child joints", ros::this_node::getName().c_str());
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no base link", ros::this_node::getName().c_str());
  }

}
//}

/* updateCameraPosition() //{ */
/* void ServoCameraPlugin::updateCameraPosition() { */
/*   ignition::math::Pose3d model_pose = model->WorldPose(); */

/*   if (compensate_tilt) {  // simulates perfect stabilization of pitch and roll angles */
/*     ignition::math::Vector3d dir = ignition::math::Vector3d(1.0, 0.0, 0.0); */
/*     dir                          = model->WorldPose().Rot() * dir; */
/*     double heading               = atan2(dir.Y(), dir.X()); */
/*     model_pose.Rot()             = ignition::math::Quaternion<double>(0.0, 0.0, heading); */
/*   } */

/*   spawn_point.Rot() = model_pose.Rot() * offset.Rot(); */
/*   spawn_point.Pos() = model_pose.Pos() + model_pose.Rot() * offset.Pos(); */
/*   ROS_INFO_THROTTLE(5.0, "[%s]: Updating camera position.", ros::this_node::getName().c_str()); */
/* } */
//}

/* OnUpdate //{ */
void ServoCameraPlugin::OnUpdate() {
  /* updateCameraPosition(); */
}
//}

GZ_REGISTER_MODEL_PLUGIN(ServoCameraPlugin)
}  // namespace gazebo
