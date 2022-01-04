#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_msgs/SpawnModel.h>

#include <ignition/math/Color.hh>

#include <sdf/sdf.hh>

#include <std_msgs/ColorRGBA.h>

namespace gazebo
{

/* Class definition */  //{
class SafetyLedPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
  physics::ModelPtr  _model;
  physics::WorldPtr  _world;
  physics::EntityPtr _model_entity;

  ros::NodeHandle *  _nh;
  transport::NodePtr _node;

  msgs::Light  _light_msg;
  msgs::Visual _visual_msg;

  event::ConnectionPtr _update_connection;

  transport::PublisherPtr _pub_light;
  transport::PublisherPtr _pub_visual;
  ros::Subscriber         _sub_heartbeat;

  ros::ServiceClient _spawn_srv;

  void OnUpdate();

private:
  ros::Timer _timer_light_spawner;
  ros::Time  _stamp_ros_init;
  void       callbackSpawnSafetyLed(const ros::TimerEvent &event);
  bool       spawnSafetyLed();

private:
  void parseParam(sdf::ElementPtr sdf, const std::string param_name, double &out_variable, const double default_value);
  bool parseParam(sdf::ElementPtr sdf, const std::string param_name, std::string &out_variable);

private:
  void callbackHeartbeat(const std_msgs::ColorRGBA::ConstPtr &heartbeat_color);

private:
  bool _initialized   = false;
  bool _light_spawned = false;

  std::string _uav_name;

  std::string _model_name;
  std::string _visual_name;
  std::string _light_name;
  std::string _link_name;
  std::string _safety_led_model;

  double _failure_duration_threshold;
  double _model_spawn_delay;

  common::Time          _time_last_update;
  ignition::math::Color _color_default;
  ignition::math::Color _color_desired;
  ignition::math::Color _color_set;

  ignition::math::Pose3d _pose_offset;
};
//}

/* Load */  //{
void SafetyLedPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  _model    = _parent;
  _world    = _parent->GetWorld();
  _uav_name = _model->GetName().c_str();

  // Load safety_led model as a string
  const std::string safety_led_model_path = ros::package::getPath("mrs_gazebo_common_resources") + "/models/safety_led/model.sdf";
  ROS_INFO("[SafetyLedPlugin] Opening safety_led model file: %s.", safety_led_model_path.c_str());
  try {
    _safety_led_model = sdf::readFile(safety_led_model_path)->ToString();
  }
  catch (...) {
    ROS_ERROR("[%s][SafetyLedPlugin] Cannot read safety_led model file!", _uav_name.c_str());
    return;
  }

  // Load name of the light and its link
  const bool has_param_model_name = parseParam(_sdf, "model_name", _model_name);
  _light_name                     = _model_name + "_light";
  _link_name                      = _model_name + "_link";
  _visual_name                    = _model_name + "_visual";

  if (!has_param_model_name) {
    ROS_ERROR("[%s][SafetyLedPlugin] Unspecified model name parameter. Quitting.", _uav_name.c_str());
    return;
  }

  // Load parameters
  parseParam(_sdf, "failure_duration_threshold", _failure_duration_threshold, 0.2);
  parseParam(_sdf, "model_spawn_delay", _model_spawn_delay, 5.0);

  double x, y, z, roll, pitch, yaw;
  parseParam(_sdf, "x", x, 0.0);
  parseParam(_sdf, "y", y, 0.0);
  parseParam(_sdf, "z", z, 0.0);
  parseParam(_sdf, "roll", roll, 0.0);
  parseParam(_sdf, "pitch", pitch, 0.0);
  parseParam(_sdf, "yaw", yaw, 0.0);
  _pose_offset.Pos() = ignition::math::Vector3<double>(x, y, z);
  _pose_offset.Rot() = ignition::math::Quaternion<double>(roll, pitch, yaw);

  // TODO: Add default color loading
  _color_default = ignition::math::Color::Red;
  _color_desired = ignition::math::Color::Red;
  _color_set     = ignition::math::Color::Red;

  _time_last_update = _world->SimTime();

  // Initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "safety_led_plugin", ros::init_options::NoSigintHandler);
  _nh = new ros::NodeHandle("~");

  _node = transport::NodePtr(new transport::Node());
  _node->Init();
  _pub_light  = _node->Advertise<gazebo::msgs::Light>("~/light/modify");
  _pub_visual = _node->Advertise<gazebo::msgs::Visual>("~/visual");

  _spawn_srv = _nh->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

  // Wait for existence of srvs/topics (do not wait for visual as it does not have to exist at all)
  _pub_light->WaitForConnection();
  _spawn_srv.waitForExistence();

  _stamp_ros_init      = ros::Time::now();
  _timer_light_spawner = _nh->createTimer(ros::Rate(2.0), &SafetyLedPlugin::callbackSpawnSafetyLed, this);

  // Subscribe to heartbeat messages
  const std::string topic = "/" + _uav_name + "/safety_led/heartbeat";
  _sub_heartbeat          = _nh->subscribe(topic, 1, &SafetyLedPlugin::callbackHeartbeat, this);
  ROS_INFO("[%s][SafetyLedPlugin] Subscribing to heartbeat on topic: %s.", _uav_name.c_str(), topic.c_str());

  // Initialize connection of this plugin and the world
  _update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&SafetyLedPlugin::OnUpdate, this));
  _initialized       = true;
}
//}

/* OnUpdate //{ */
void SafetyLedPlugin::OnUpdate() {

  if (!_initialized || !_light_spawned) {
    return;
  }

  // | ------------------------ Set pose ------------------------ |

  const ignition::math::Pose3d model_pose = _model->WorldPose();

  ignition::math::Pose3d shifted_pose;
  shifted_pose.Rot() = model_pose.Rot() * _pose_offset.Rot();
  shifted_pose.Pos() = model_pose.Pos() + model_pose.Rot() * _pose_offset.Pos();

  _model_entity->SetWorldPose(shifted_pose);

  // | ------------------------ Set color ----------------------- |

  const common::Time stamp = _world->SimTime();

  // Override color if heartbeat msg have not arrived within safety duration limit
  if (stamp - _time_last_update > _failure_duration_threshold) {
    _color_desired = _color_default;
  }

  // Do not update model color if change in color will not occur
  if (_color_desired == _color_set || _color_desired == ignition::math::Color::Black) {
    return;
  }

  // Update visual of the LED
  msgs::Set(_visual_msg.mutable_material()->mutable_diffuse(), _color_desired);
  msgs::Set(_visual_msg.mutable_material()->mutable_emissive(), _color_desired);
  msgs::Set(_visual_msg.mutable_material()->mutable_specular(), _color_desired);
  msgs::Set(_visual_msg.mutable_material()->mutable_ambient(), _color_desired);
  _pub_visual->Publish(_visual_msg);

  // Update light of the LED
  msgs::Set(_light_msg.mutable_diffuse(), _color_desired);
  msgs::Set(_light_msg.mutable_specular(), _color_desired);
  _pub_light->Publish(_light_msg);

  _color_set = _color_desired;
}
//}

/* parseParam //{ */
void SafetyLedPlugin::parseParam(sdf::ElementPtr sdf, const std::string param_name, double &out_variable, const double default_value) {

  if (sdf->HasElement(param_name)) {
    out_variable = sdf->Get<double>(param_name);
  } else {
    out_variable = default_value;
    ROS_WARN("[%s][Parachute]: param \"%s\" not defined! Used default value: %.4f", _uav_name.c_str(), param_name.c_str(), default_value);
  }
}

bool SafetyLedPlugin::parseParam(sdf::ElementPtr sdf, const std::string param_name, std::string &out_variable) {

  if (sdf->HasElement(param_name)) {
    out_variable = sdf->Get<std::string>(param_name);
  } else {
    ROS_WARN("[%s][Parachute]: param \"%s\" not defined!", _uav_name.c_str(), param_name.c_str());
    return false;
  }

  return true;
}
//}

/*//{ callbackHeartbeat() */
void SafetyLedPlugin::callbackHeartbeat(const std_msgs::ColorRGBA::ConstPtr &heartbeat_color) {

  if (!_initialized || !_light_spawned) {
    return;
  }

  _time_last_update = _world->SimTime();
  _color_desired.Set(heartbeat_color->r, heartbeat_color->g, heartbeat_color->b, heartbeat_color->a);
}
/*//}*/

/* callbackSpawnSafetyLed //{ */
void SafetyLedPlugin::callbackSpawnSafetyLed(const ros::TimerEvent &event) {

  if (!_initialized || _light_spawned) {
    return;
  }

  // Give spawning small booting delay
  if ((event.current_real - _stamp_ros_init).toSec() < _model_spawn_delay) {
    return;
  }

  const bool spawned = spawnSafetyLed();

  if (spawned) {

    ROS_INFO("[%s][SafetyLedPlugin] Safety led successfully spawned.", _uav_name.c_str());

    _timer_light_spawner.stop();

    // Get light model entity
    _model_entity = _world->EntityByName(_uav_name + "_" + _model_name);

    // Set scoped name of the light
    _light_msg.set_name(_model_entity->GetScopedName() + "::" + _link_name + "::" + _light_name);

    // Set scoped names of the light visual
    _visual_msg.set_parent_name(_model_entity->GetScopedName() + "::" + _link_name);
    _visual_msg.set_name(_model_entity->GetScopedName() + "::" + _link_name + "::" + _visual_name);

    // Call first visual change (visual is white by default)
    msgs::Set(_visual_msg.mutable_material()->mutable_diffuse(), _color_default);
    msgs::Set(_visual_msg.mutable_material()->mutable_emissive(), _color_default);
    msgs::Set(_visual_msg.mutable_material()->mutable_specular(), _color_default);
    msgs::Set(_visual_msg.mutable_material()->mutable_ambient(), _color_default);
    _pub_visual->Publish(_visual_msg);
  }

  _light_spawned = spawned;
}
//}

/* spawnSafetyLed //{ */
bool SafetyLedPlugin::spawnSafetyLed() {

  ROS_INFO("[%s][SafetyLedPlugin] Spawning safety_led.", _uav_name.c_str());

  const auto             model_pose = _model->WorldPose();
  ignition::math::Pose3d shifted_pose;
  shifted_pose.Rot() = model_pose.Rot() * _pose_offset.Rot();
  shifted_pose.Pos() = model_pose.Pos() + model_pose.Rot() * _pose_offset.Pos();

  gazebo_msgs::SpawnModel spawn_call;
  spawn_call.request.initial_pose.position.x    = shifted_pose.Pos().X();
  spawn_call.request.initial_pose.position.y    = shifted_pose.Pos().Y();
  spawn_call.request.initial_pose.position.z    = shifted_pose.Pos().Z();
  spawn_call.request.initial_pose.orientation.w = shifted_pose.Rot().W();
  spawn_call.request.initial_pose.orientation.x = shifted_pose.Rot().X();
  spawn_call.request.initial_pose.orientation.y = shifted_pose.Rot().Y();
  spawn_call.request.initial_pose.orientation.z = shifted_pose.Rot().Z();
  spawn_call.request.model_name                 = _uav_name + "_" + _model_name;
  spawn_call.request.model_xml                  = _safety_led_model;
  _spawn_srv.call(spawn_call);

  if (!spawn_call.response.success) {
    ROS_ERROR("[%s][SafetyLedPlugin] Failed safety led spawning response: %s.", _uav_name.c_str(), spawn_call.response.status_message.c_str());
  }

  return spawn_call.response.success;
}
//}

GZ_REGISTER_MODEL_PLUGIN(SafetyLedPlugin)
}  // namespace gazebo
