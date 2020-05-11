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
  ros::ServiceClient spawn_srv, change_state_srv;
  ros::Subscriber    pitch_sub;
  ros::Timer         camera_timer;

  void pitchCallback(const std_msgs::Float32 &desired_pitch);

  void updateCameraPosition();
  void OnUpdate();
  bool spawnCamera();
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
  double                 desired_pitch;
  double                 spawn_delay_sec = 5.0;
  int                    spawn_delay_remaining_loops;
  bool                   camera_spawned;

  event::ConnectionPtr updateConnection;
};
//}

/* Load */  //{
void ServoCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  ROS_INFO("[%s]: Servo camera plugin started.", ros::this_node::getName().c_str());
  model       = _parent;
  parent_name  = model->GetName().c_str();
  offset_yaw   = 0.0;
  offset_pitch = 0.0;
  offset_roll  = 0.0;

  std::stringstream camera_model_path;

  if (_sdf->HasElement("camera_type")) {
    camera_type = _sdf->Get<std::string>("camera_type");
  } else {
    ROS_WARN("[%s][Servo camera]: Camera type not defined. Setting default type.", parent_name.c_str());
    camera_type = "servo_camera";
  }

  camera_model_path << ros::package::getPath("mrs_gazebo_common_resources") << "/models/" << camera_type
                    << "/model.sdf";
  ROS_INFO("[%s][Servo camera]: trying to open file %s", parent_name.c_str(), camera_model_path.str().c_str());
  try {
    camera_model = sdf::readFile(camera_model_path.str())->ToString();
    ROS_INFO("[%s][Servo camera]: model loaded successfully", parent_name.c_str());
  }
  catch (...) {
    ROS_FATAL("[%s][Servo camera]: cannot read camera model file!", parent_name.c_str());
  }

  if (_sdf->HasElement("offset_x")) {
    offset.Pos().X() = _sdf->Get<double>("offset_x");
  } else {
    ROS_WARN("[%s][Servo camera]: Spawn offset_x not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_y")) {
    offset.Pos().Y() = _sdf->Get<double>("offset_y");
  } else {
    ROS_WARN("[%s][Servo camera]: Spawn offset_y not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_z")) {
    offset.Pos().Z() = _sdf->Get<double>("offset_z");
  } else {
    ROS_WARN("[%s][Servo camera]: Spawn offset_z not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_pitch")) {
    offset_pitch = _sdf->Get<double>("offset_pitch");
  } else {
    ROS_WARN("[%s][Servo camera]: Spawn offset_pitch not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_pitch")) {
    offset_roll = _sdf->Get<double>("offset_roll");
  } else {
    ROS_WARN("[%s][Servo camera]: Spawn offset_roll not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_yaw")) {
    offset_yaw = _sdf->Get<double>("offset_yaw");
  } else {
    ROS_WARN("[%s][Servo camera]: Spawn offset_yaw not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("max_pitch_rate")) {
    max_pitch_rate = _sdf->Get<double>("max_pitch_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Update_rate not defined. Setting to defalt value.", parent_name.c_str());
    max_pitch_rate = 10;
  }
  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->Get<double>("update_rate");
  } else {
    ROS_WARN("[%s][Servo camera]: Update_rate not defined. Setting to defalt value.", parent_name.c_str());
    update_rate = 30;
  }
  if (_sdf->HasElement("joint_name")) {
    joint_name = _sdf->Get<std::string>("joint_name");
  } else {
    ROS_WARN("[%s][Servo camera]: Joint name not defined.", parent_name.c_str());
  }

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

  spawn_delay_remaining_loops = spawn_delay_sec * update_rate;
  camera_spawned              = false;

  camera_timer = nh->createTimer(ros::Rate(update_rate), &ServoCameraPlugin::cameraTimerCallback, this);

  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ServoCameraPlugin::OnUpdate, this));

  ROS_INFO("[%s][Servo camera]: Servo camera initialized.", parent_name.c_str());
}
//}

/* convertEulerToQuaternion() //{ */
void ServoCameraPlugin::convertEulerToQuaternion(double roll_offset, double pitch_offset, double yaw_offset) {
  offset.Rot() = ignition::math::Quaternion<double>(roll_offset, pitch_offset, yaw_offset);
}
//}

/* spawnCamera() */  //{
bool ServoCameraPlugin::spawnCamera() {
  std::stringstream       ss;
  gazebo_msgs::SpawnModel spawn_call;
  spawn_call.request.reference_frame = parent_name;
  spawn_call.request.model_xml       = camera_model;
  ss.str(std::string());
  ss << parent_name << "_camera_panel";
  spawn_call.request.model_name = ss.str().c_str();
  spawn_srv.call(spawn_call);
  if (spawn_call.response.success) {
    ROS_INFO("[%s][servo_camera]: Camera spawned.", parent_name.c_str());
    camera_spawned = true;
  } else {
    ROS_WARN("[%s]: Spawn not succesfull, msg: %s", ros::this_node::getName().c_str(), spawn_call.response.status_message.c_str());
  }
  return spawn_call.response.success;
}
//}

/* cameraTimerCallback */  //{
void ServoCameraPlugin::cameraTimerCallback(const ros::TimerEvent &event) {
  if (!camera_spawned) {
    if (spawn_delay_remaining_loops < 0) {
      spawnCamera();
    } else {
      spawn_delay_remaining_loops--;
    }
  }
  if (camera_spawned) {
    moveCamera();
  }
}
//}

/* pitchCallback */  //{
void ServoCameraPlugin::pitchCallback(const std_msgs::Float32 &msg) {
  /* ROS_INFO("[%s][Servo camera]: Change camera pitch!", parent_name.c_str()); */
  desired_pitch = fmod(msg.data + 2 * M_PI, 2 * M_PI);
}
//}

/* moveCamera() //{ */
void ServoCameraPlugin::moveCamera() {
  std::stringstream ss;
  ss << parent_name << "_camera_panel";
  gazebo_msgs::SetModelState srv;
  srv.request.model_state.twist.linear.x     = 0.0;
  srv.request.model_state.twist.linear.y     = 0.0;
  srv.request.model_state.twist.linear.z     = 0.0;
  srv.request.model_state.model_name         = ss.str().c_str();
  srv.request.model_state.pose.position.x    = spawn_point.Pos().X();
  srv.request.model_state.pose.position.y    = spawn_point.Pos().Y();
  srv.request.model_state.pose.position.z    = spawn_point.Pos().Z();
  srv.request.model_state.pose.orientation.x = spawn_point.Rot().X();
  srv.request.model_state.pose.orientation.y = spawn_point.Rot().Y();
  srv.request.model_state.pose.orientation.z = spawn_point.Rot().Z();
  srv.request.model_state.pose.orientation.w = spawn_point.Rot().W();

  double abs_diff = abs(offset_pitch - desired_pitch);
  double min_diff = abs_diff > M_PI ? 2 * M_PI - abs_diff : abs_diff;

  if (min_diff > 1e-5) {
    double dir = desired_pitch > offset_pitch ? 1.0 : -1.0;
    dir        = abs(min_diff - abs_diff) > 1e-5 ? -1.0 * dir : dir;
    offset_pitch += dir * fmin(abs(desired_pitch - offset_pitch), max_pitch_rate / update_rate);
    convertEulerToQuaternion(offset_roll, offset_pitch, offset_yaw);
    offset_pitch = fmod(offset_pitch + 2 * M_PI, 2 * M_PI);
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
        model->GetLink("base_link")->GetChildJoints()[index]->SetPosition(0, offset_pitch);
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Servo camera macro joint doesn't found.", ros::this_node::getName().c_str());
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Base link has no child joints", ros::this_node::getName().c_str());
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Model has no base link", ros::this_node::getName().c_str());
  }

  change_state_srv.call(srv);
  if (!srv.response.success) {
    ROS_WARN("[%s]: Unsuccessful model state change.", ros::this_node::getName().c_str());
  }
}
//}

/* updateCameraPosition() //{ */
void ServoCameraPlugin::updateCameraPosition() {
  spawn_point.Rot() = model->WorldPose().Rot() * offset.Rot();
  spawn_point.Pos() = model->WorldPose().Pos() + model->WorldPose().Rot() * offset.Pos();
}
//}

/* OnUpdate //{ */
void ServoCameraPlugin::OnUpdate() {
  updateCameraPosition();
}
//}

GZ_REGISTER_MODEL_PLUGIN(ServoCameraPlugin)
}  // namespace gazebo
