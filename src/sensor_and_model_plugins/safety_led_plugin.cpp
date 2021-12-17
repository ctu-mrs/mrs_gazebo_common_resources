#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

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
  physics::ModelPtr _model;
  physics::WorldPtr _world;
  physics::LinkPtr  _light_link;

  ros::NodeHandle *  _nh;
  transport::NodePtr _node;
  msgs::Light        _light_msg;

  transport::PublisherPtr _pub_light;
  ros::Subscriber         _sub_heartbeat;

  void OnUpdate();

private:
  void parseParam(sdf::ElementPtr sdf, const std::string param_name, double &out_variable, const double default_value);

private:
  physics::LinkPtr findLinkForLight(const physics::ModelPtr &model, const std::string &light_name, const std::string &link_name);
  void             callbackHeartbeat(const std_msgs::ColorRGBA::ConstPtr &heartbeat_color);

private:
  bool _initialized = false;

  std::string _parent_name;
  std::string _uav_name;

  std::string _light_name;
  std::string _link_name;

  double _range = 0.5;
  double _safety_duration;

  common::Time          _time_last_update;
  ignition::math::Color _color_default;
  ignition::math::Color _color_desired;
  ignition::math::Color _color_set;
};
//}

/* Load */  //{
void SafetyLedPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  _model       = _parent;
  _world       = _parent->GetWorld();
  _parent_name = _model->GetName().c_str();

  if (_sdf->HasElement("light_name") && _sdf->HasElement("link_name")) {
    _light_name = _sdf->Get<std::string>("light_name");
    _link_name  = _sdf->Get<std::string>("link_name");
  } else {
    ROS_ERROR("[%s][SafetyLedPlugin] No <light_name> (name of the safety light) or <link_name> in params.", _uav_name.c_str());
    return;
  }

  _light_link = findLinkForLight(_model, _light_name, _link_name);

  if (!_light_link) {
    ROS_ERROR("[%s][SafetyLedPlugin] No link (%s) for light (%s) was found.", _uav_name.c_str(), _link_name.c_str(), _light_name.c_str());
    return;
  }

  parseParam(_sdf, "air_density", _safety_duration, 0.2);
  // TODO: add default color loading

  // Find attenuation range of the safety led light
  if (_light_link->GetSDF()->HasElement("light")) {
    auto sdfLight = _light_link->GetSDF()->GetElement("light");
    while (sdfLight) {
      if (sdfLight->Get<std::string>("name") == _light_name) {
        _range = sdfLight->GetElement("attenuation")->Get<double>("range");
        break;
      }
      sdfLight = sdfLight->GetNextElement("light");
    }
  }

  _color_default.Set(1, 0, 0, 1);
  _color_desired.Set(1, 0, 0, 1);
  _color_set.Set(1, 0, 0, 1);
  _time_last_update = _world->SimTime();

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "safety_led_plugin", ros::init_options::NoSigintHandler);
  _nh = new ros::NodeHandle("~");

  _node = transport::NodePtr(new transport::Node());
  _node->Init();
  _pub_light = _node->Advertise<gazebo::msgs::Light>("~/light/modify");
  _pub_light->WaitForConnection();

  // TODO: verify the name is correctly set
  _light_msg.set_name(_light_link->GetScopedName() + "::" + _light_name);
  _light_msg.set_range(float(_range));

  const std::string topic = "/" + _uav_name + "/safety_led/heartbeat";
  _sub_heartbeat          = _nh->subscribe(topic, 1, &SafetyLedPlugin::callbackHeartbeat, this);
  ROS_INFO("[%s][SafetyLedPlugin] Subscribing to heartbeat on topic: %s.", _uav_name.c_str(), topic.c_str());


  _initialized = true;
}
//}

/*//{ findLinkForLight() */
physics::LinkPtr SafetyLedPlugin::findLinkForLight(const physics::ModelPtr &model, const std::string &light_name, const std::string &link_name) {

  auto childLink = model->GetChildLink(link_name);
  if (childLink && childLink->GetSDF()->HasElement("light")) {
    auto sdfLight = childLink->GetSDF()->GetElement("light");

    while (sdfLight) {
      if (sdfLight->Get<std::string>("name") == light_name) {
        return childLink;
      }
      sdfLight = sdfLight->GetNextElement("light");
    }
  }

  for (const auto &model : model->NestedModels()) {
    const auto foundLink = this->findLinkForLight(model, light_name, link_name);
    if (foundLink) {
      return foundLink;
    }
  }

  return nullptr;
}
/*//}*/

/* OnUpdate //{ */
void SafetyLedPlugin::OnUpdate() {

  const common::Time stamp = _world->SimTime();

  // Override color if heartbeat msg have not arrived within safety-duration limit
  if (stamp - _time_last_update > _safety_duration) {
    _color_desired = _color_default;
  }

  if (_color_desired == _color_set || _color_desired == ignition::math::Color::Black) {
    return;
  }

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
//}

/*//{ callbackHeartbeat() */
void SafetyLedPlugin::callbackHeartbeat(const std_msgs::ColorRGBA::ConstPtr &heartbeat_color) {

  if (!_initialized) {
    return;
  }

  _time_last_update = _world->SimTime();
  _color_desired.Set(heartbeat_color->r, heartbeat_color->g, heartbeat_color->b, heartbeat_color->a);
}
/*//}*/

GZ_REGISTER_MODEL_PLUGIN(SafetyLedPlugin)
}  // namespace gazebo
