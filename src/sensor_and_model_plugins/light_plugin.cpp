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
class GazeboLightPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

private:
  physics::ModelPtr  model;
  ros::NodeHandle *  nh;
  ros::ServiceServer trigger_srv;
  ros::ServiceClient spawn_srv, delete_srv, change_state_srv;
  ros::Subscriber    pitch_sub;
  ros::Timer         light_timer;

  bool triggerCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  void pitchCallback(const std_msgs::Float32 &desired_pitch);
  void odomCallback(const nav_msgs::OdometryPtr &odom);

  void updateLightPosition();
  void OnUpdate();
  bool spawnLight();
  void lightTimerCallback(const ros::TimerEvent &event);
  void moveLight(bool move_it_away);
  void moveLightPanelOnly();
  void convertEulerToQuaternion(double roll_offset, double pitch_offset, double yaw_offset);

  std::string            light_model;
  std::string            parent_name;
  ignition::math::Pose3d offset;
  ignition::math::Pose3d spawn_point;
  bool                   light_spawned;
  bool                   light_attached;
  bool                   initial_on;
  double                 offset_roll;
  double                 offset_pitch;
  double                 offset_yaw;
  double                 update_rate;
  double                 max_pitch_rate;
  double                 desired_pitch;
  double                 spawn_delay_sec = 5.0;
  int                    spawn_delay_remaining_loops;

  event::ConnectionPtr updateConnection;
};
//}

/* Load */  //{
void GazeboLightPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  ROS_INFO("[%s]: Light gazebo plugin started.", ros::this_node::getName().c_str());
  model        = _parent;
  parent_name  = model->GetName().c_str();
  offset_yaw   = 0.0;
  offset_pitch = 0.0;
  offset_roll  = 0.0;

  std::stringstream light_model_path;
  light_model_path << ros::package::getPath("mrs_gazebo_common_resources") << "/models/light_panel/model.sdf";
  ROS_INFO("[%s][Light]: trying to open file %s", parent_name.c_str(), light_model_path.str().c_str());
  try {
    light_model = sdf::readFile(light_model_path.str())->ToString();
    ROS_INFO("[%s][Light]: model loaded successfully", parent_name.c_str());
  }
  catch (...) {
    ROS_FATAL("[%s][Light]: cannot read light model file!", parent_name.c_str());
  }

  if (_sdf->HasElement("offset_x")) {
    offset.Pos().X() = _sdf->Get<double>("offset_x");
  } else {
    ROS_WARN("[%s][Light]: Spawn offset_x not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_y")) {
    offset.Pos().Y() = _sdf->Get<double>("offset_y");
  } else {
    ROS_WARN("[%s][Light]: Spawn offset_y not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_z")) {
    offset.Pos().Z() = _sdf->Get<double>("offset_z");
  } else {
    ROS_WARN("[%s][Light]: Spawn offset_z not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_pitch")) {
    offset_pitch = _sdf->Get<double>("offset_pitch");
  } else {
    ROS_WARN("[%s][Light]: Spawn offset_pitch not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_pitch")) {
    offset_roll = _sdf->Get<double>("offset_roll");
  } else {
    ROS_WARN("[%s][Light]: Spawn offset_roll not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_yaw")) {
    offset_yaw = _sdf->Get<double>("offset_yaw");
  } else {
    ROS_WARN("[%s][Light]: Spawn offset_yaw not defined.", parent_name.c_str());
  }
  if (_sdf->HasElement("max_pitch_rate")) {
    max_pitch_rate = _sdf->Get<double>("max_pitch_rate");
  } else {
    ROS_WARN("[%s][Light]: Update_rate not defined. Setting to defalt value.", parent_name.c_str());
    max_pitch_rate = 10;
  }
  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->Get<double>("update_rate");
  } else {
    ROS_WARN("[%s][Light]: Update_rate not defined. Setting to defalt value.", parent_name.c_str());
    update_rate = 30;
  }
  if (_sdf->HasElement("initial_on")) {
    initial_on = _sdf->Get<bool>("initial_on");
  } else {
    ROS_WARN("[%s][Light]: Initial_on not defined. Setting to defalt value.", parent_name.c_str());
    initial_on = true;
  }

  convertEulerToQuaternion(offset_roll, offset_pitch, offset_yaw);

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "light_gazebo_plugin", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle("~");

  // connect to gazebo model control services
  spawn_srv        = nh->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  delete_srv       = nh->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  change_state_srv = nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboLightPlugin::OnUpdate, this));

  // create ROS services to control the light
  std::stringstream ss;
  ss << "/" << parent_name << "/light/set_pitch";
  pitch_sub = nh->subscribe(ss.str().c_str(), 1, &GazeboLightPlugin::pitchCallback, this);
  ss.str(std::string());
  ss << "/" << parent_name << "/light/trigger";
  trigger_srv = nh->advertiseService(ss.str().c_str(), &GazeboLightPlugin::triggerCallback, this);

  light_spawned  = false;
  light_attached = false;

  light_timer = nh->createTimer(ros::Rate(update_rate), &GazeboLightPlugin::lightTimerCallback, this);

  if (initial_on) {
    spawn_delay_remaining_loops = spawn_delay_sec * update_rate;
  }

  ROS_INFO("[%s][Light]: Light initialized.", parent_name.c_str());
}
//}

/* convertEulerToQuaternion() //{ */
void GazeboLightPlugin::convertEulerToQuaternion(double roll_offset, double pitch_offset, double yaw_offset) {
  offset.Rot() = ignition::math::Quaternion<double>(roll_offset, pitch_offset, yaw_offset);
}
//}

/* spawnLight() */  //{
bool GazeboLightPlugin::spawnLight() {
  std::stringstream       ss;
  gazebo_msgs::SpawnModel spawn_call;
  spawn_call.request.reference_frame = parent_name;
  spawn_call.request.model_xml       = light_model;
  ss.str(std::string());
  ss << parent_name << "_light_panel";
  spawn_call.request.model_name = ss.str().c_str();
  spawn_srv.call(spawn_call);
  if (spawn_call.response.success) {
    ROS_INFO("[%s][Light]: Light spawned.", parent_name.c_str());
    light_spawned  = true;
    light_attached = true;
  } else {
    ROS_WARN("[%s]: Spawn not succesfull, msg: %s", ros::this_node::getName().c_str(), spawn_call.response.status_message.c_str());
  }
  return spawn_call.response.success;
}
//}

/* triggerCallback() */  //{
bool GazeboLightPlugin::triggerCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Light]: Trigger light!", parent_name.c_str());

  if (req.data) {
    if (!light_spawned) {
      spawnLight();
    } else {
      light_attached = true;
      moveLight(false);
    }
  } else {
    if (light_spawned) {
      light_attached = false;
      moveLight(true);
    }
  }

  res.success = true;
  return res.success;
}
//}

/* lightTimerCallback */  //{
void GazeboLightPlugin::lightTimerCallback(const ros::TimerEvent &event) {
  if (initial_on && !light_spawned) {
    if (spawn_delay_remaining_loops < 0) {
      spawnLight();
    } else {
      spawn_delay_remaining_loops--;
    }
  }
  if (light_attached && light_spawned) {
    moveLight(false);
  } else {
    moveLightPanelOnly();
  }
}
//}

/* pitchCallback */  //{
void GazeboLightPlugin::pitchCallback(const std_msgs::Float32 &msg) {
  desired_pitch = fmod(msg.data + 2 * M_PI, 2 * M_PI);
}
//}

/* moveLight() //{ */
void GazeboLightPlugin::moveLight(bool move_it_away) {
  /* ROS_INFO_THROTTLE(1, "[%s]: desired_pitch = %.3f, offset_pitch = %.3f", ros::this_node::getName().c_str(), desired_pitch, offset_pitch); */
  std::stringstream ss;
  ss << parent_name << "_light_panel";
  gazebo_msgs::SetModelState srv;
  srv.request.model_state.twist.linear.x = 0.0;
  srv.request.model_state.twist.linear.y = 0.0;
  srv.request.model_state.twist.linear.z = 0.0;
  srv.request.model_state.model_name     = ss.str().c_str();
  if (move_it_away) {
    srv.request.model_state.pose.position.x = 100;
    srv.request.model_state.pose.position.y = 100;
    srv.request.model_state.pose.position.z = 1;
  } else {
    double abs_diff = abs(offset_pitch - desired_pitch);
    double min_diff = abs_diff > M_PI ? 2 * M_PI - abs_diff : abs_diff;
    if (min_diff > 1e-5) {
      double dir = desired_pitch > offset_pitch ? 1.0 : -1.0;
      dir        = abs(min_diff - abs_diff) > 1e-5 ? -1.0 * dir : dir;
      offset_pitch += dir * fmin(abs(desired_pitch - offset_pitch), max_pitch_rate / update_rate);
      convertEulerToQuaternion(offset_roll, offset_pitch, offset_yaw);
      offset_pitch = fmod(offset_pitch + 2 * M_PI, 2 * M_PI);
    }
    srv.request.model_state.pose.position.x    = spawn_point.Pos().X();
    srv.request.model_state.pose.position.y    = spawn_point.Pos().Y();
    srv.request.model_state.pose.position.z    = spawn_point.Pos().Z();
    srv.request.model_state.pose.orientation.x = spawn_point.Rot().X();
    srv.request.model_state.pose.orientation.y = spawn_point.Rot().Y();
    srv.request.model_state.pose.orientation.z = spawn_point.Rot().Z();
    srv.request.model_state.pose.orientation.w = spawn_point.Rot().W();

    if (model->GetLink("base_link") != NULL) {
      if (model->GetLink("base_link")->GetChildJoints().size() != 0) {
        int index = -1;
        for (size_t i = 0; i < model->GetLink("base_link")->GetChildJoints().size(); i++) {
          if (model->GetLink("base_link")->GetChildJoints()[i]->GetName() == "light_macro_joint") {
            index = i;
            break;
          }
        }
        if (index != -1) {
          model->GetLink("base_link")->GetChildJoints()[index]->SetPosition(0, offset_pitch);
        } else {
          ROS_WARN_THROTTLE(1.0, "[%s]: Light macro joint doesn't found.", ros::this_node::getName().c_str());
        }
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Base link has no child joints", ros::this_node::getName().c_str());
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Model has no base link", ros::this_node::getName().c_str());
    }
  }

  change_state_srv.call(srv);
  if (!srv.response.success) {
    ROS_WARN("[%s]: Unsuccessful model state change.", ros::this_node::getName().c_str());
  }
}
//}

/* moveLightPanelOnly() //{ */
void GazeboLightPlugin::moveLightPanelOnly() {
  double abs_diff = abs(offset_pitch - desired_pitch);
  double min_diff = abs_diff > M_PI ? 2 * M_PI - abs_diff : abs_diff;
  if (min_diff > 1e-5) {
    double dir = desired_pitch > offset_pitch ? 1.0 : -1.0;
    dir        = abs(min_diff - abs_diff) > 1e-5 ? -1.0 * dir : dir;
    offset_pitch += dir * fmin(abs(desired_pitch - offset_pitch), max_pitch_rate / update_rate);
    convertEulerToQuaternion(offset_roll, offset_pitch, offset_yaw);
    offset_pitch = fmod(offset_pitch + 2 * M_PI, 2 * M_PI);

    if (model->GetLink("base_link") != NULL) {
      if (model->GetLink("base_link")->GetChildJoints().size() != 0) {
        int index = -1;
        for (size_t i = 0; i < model->GetLink("base_link")->GetChildJoints().size(); i++) {
          if (model->GetLink("base_link")->GetChildJoints()[i]->GetName() == "light_macro_joint") {
            index = i;
            break;
          }
        }
        if (index != -1) {
          model->GetLink("base_link")->GetChildJoints()[index]->SetPosition(0, offset_pitch);
        } else {
          ROS_WARN_THROTTLE(1.0, "[%s]: Light macro joint doesn't found.", ros::this_node::getName().c_str());
        }
      } else {
        ROS_WARN_THROTTLE(1.0, "[%s]: Base link has no child joints", ros::this_node::getName().c_str());
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Model has no base link", ros::this_node::getName().c_str());
    }
  }
}
//}

/* updateLightPosition() //{ */
void GazeboLightPlugin::updateLightPosition() {
  spawn_point.Rot() = model->WorldPose().Rot() * offset.Rot();
  spawn_point.Pos() = model->WorldPose().Pos() + model->WorldPose().Rot() * offset.Pos();
}
//}

/* OnUpdate //{ */
void GazeboLightPlugin::OnUpdate() {
  updateLightPosition();
}
//}

GZ_REGISTER_MODEL_PLUGIN(GazeboLightPlugin)
}  // namespace gazebo
