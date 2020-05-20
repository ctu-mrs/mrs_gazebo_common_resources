#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/physics/Model.hh>

#include <sdf/sdf.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <std_srvs/Trigger.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include <ignition/math.hh>

namespace gazebo
{

/* Class definition */  //{
class ParachutePlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

private:
  bool                   deployed = false;
  physics::ModelPtr      model_;
  std::string            uav_name;
  ignition::math::Pose3d uav_pose;
  ignition::math::Pose3d parachute_pose;
  std::string            parachute_model_xml;

  // stuff loaded as params
  double                   air_density;
  double                   drag_coeff;
  double                   cross_section;
  ignition::math::Vector3d parachute_offset;

  ros::NodeHandle *nh_;

  // services
  ros::ServiceServer deploy_srv, reset_srv;
  ros::ServiceClient spawn_srv, delete_srv;

  bool deployCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  // internal updating
  event::ConnectionPtr updateConnection;

  void OnUpdate();
  void updateDeployedModelPosition();
  void updatePhysics();

  bool spawnDeployedModel();
  bool deleteDeployedModel();

  void parseParam(sdf::ElementPtr sdf_, std::string param_name, double &out_variable, double default_value);
};
//}

/* parseParam //{ */
void ParachutePlugin::parseParam(sdf::ElementPtr sdf_, std::string param_name, double &out_variable, double default_value) {

  if (sdf_->HasElement(param_name)) {
    out_variable = sdf_->Get<double>(param_name);
  } else {
    out_variable = default_value;
    ROS_WARN("[%s][Parachute]: param \"%s\" not defined! Used default value: %.4f", uav_name.c_str(), param_name.c_str(), default_value);
  }
}
//}

/* Load */  //{
void ParachutePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  model_   = _parent;
  uav_name = model_->GetName().c_str();

  std::stringstream parachute_model_path;
  parachute_model_path << ros::package::getPath("mrs_gazebo_common_resources") << "/models/deployed_parachute/model.sdf";
  ROS_INFO("[%s][Parachute]: trying to open file %s", uav_name.c_str(), parachute_model_path.str().c_str());
  try {
    parachute_model_xml = sdf::readFile(parachute_model_path.str())->ToString();
    ROS_INFO("[%s][Parachute]: parachute model loaded successfully", uav_name.c_str());
  }
  catch (...) {
    ROS_FATAL("[%s][Parachute]: cannot read parachute model file!", uav_name.c_str());
  }
  parseParam(_sdf, "air_density", air_density, 1.225);
  parseParam(_sdf, "drag_coeff", drag_coeff, 500);
  parseParam(_sdf, "cross_section", cross_section, 0.25);
  parseParam(_sdf, "offset_x", parachute_offset.X(), 0.0);
  parseParam(_sdf, "offset_y", parachute_offset.Y(), 0.0);
  parseParam(_sdf, "offset_z", parachute_offset.Z(), 0.0);

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "parachute_plugin", ros::init_options::NoSigintHandler);
  nh_ = new ros::NodeHandle("~");

  // connect to gazebo model control services
  spawn_srv  = nh_->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  delete_srv = nh_->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ParachutePlugin::OnUpdate, this));

  std::stringstream ss;
  // create ROS services to control the parachute
  ss << "/" << uav_name << "/parachute/deploy";
  deploy_srv = nh_->advertiseService(ss.str().c_str(), &ParachutePlugin::deployCallback, this);
  ss.str(std::string());
  ss << "/" << uav_name << "/parachute/reset";
  reset_srv = nh_->advertiseService(ss.str().c_str(), &ParachutePlugin::resetCallback, this);

  ROS_INFO("[%s][Parachute]: Plugin initialized", uav_name.c_str());
}
//}

/* deployCallback */  //{
bool ParachutePlugin::deployCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  ROS_INFO("[%s][Parachute]: Deploy!", uav_name.c_str());
  res.success = spawnDeployedModel();
  if (res.success) {
    deployed = true;
  }
  return res.success;
}

//}

/* resetCallback */  //{
bool ParachutePlugin::resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  ROS_INFO("[%s][Parachute]: Reset!", uav_name.c_str());
  res.success = deleteDeployedModel();
  if (res.success) {
    deployed = false;
  }
  return res.success;
}

//}

/* OnUpdate //{ */
void ParachutePlugin::OnUpdate() {
  if (deployed) {
    updateDeployedModelPosition();
    updatePhysics();
  }
}
//}

/* updateDeployedModelPosition //{ */
void ParachutePlugin::updateDeployedModelPosition() {
  physics::ModelPtr parachute_model_;
  try {
    parachute_model_ = model_->GetWorld()->ModelByName("deployed_parachute");
  }
  catch (...) {
    ROS_ERROR("[%s][Parachute]: Deployed parachute model not found!", uav_name.c_str());
    return;
  }

  parachute_pose.Pos() = model_->WorldPose().Pos() + model_->WorldPose().Rot() * parachute_offset;
  parachute_pose.Rot() = model_->WorldPose().Rot();
  parachute_model_->SetWorldPose(parachute_pose);
}
//}

/* updatePhysics //{ */
void ParachutePlugin::updatePhysics() {
  physics::LinkPtr link_ = model_->GetLink("base_link");

  ignition::math::Vector3d air_velocity   = -(link_->WorldLinearVel());
  double                   air_vel_mag    = air_velocity.Length();
  double                   drag_force_mag = 0.5 * air_density * air_vel_mag * air_vel_mag * drag_coeff * cross_section;
  ignition::math::Vector3d drag_force     = air_velocity.Normalized() * drag_force_mag;
  link_->SetForce(drag_force);

  ignition::math::Vector3d rpy = model_->WorldPose().Rot().Euler();
  ignition::math::Vector3d adhoc_drag_torque;
  adhoc_drag_torque.X() = -0.7 * sin(rpy.X());
  adhoc_drag_torque.Y() = -0.7 * sin(rpy.Y());
  link_->SetTorque(adhoc_drag_torque);
  ROS_INFO_STREAM("Force applied: " << drag_force);
  ROS_INFO_STREAM("Torque applied: " << drag_force);
  ROS_INFO_STREAM("Roll: " << rpy.X() << ", Pitch: " << rpy.Y());
}
//}

/* spawnDeployedModel //{ */
bool ParachutePlugin::spawnDeployedModel() {
  ROS_INFO("[%s][Parachute]: Spawning deployed model", uav_name.c_str());
  gazebo_msgs::SpawnModel spawn_call;
  spawn_call.request.initial_pose.position.x    = model_->WorldPose().Pos().X();
  spawn_call.request.initial_pose.position.y    = model_->WorldPose().Pos().Y();
  spawn_call.request.initial_pose.position.z    = model_->WorldPose().Pos().Z();
  spawn_call.request.initial_pose.orientation.w = model_->WorldPose().Rot().W();
  spawn_call.request.initial_pose.orientation.x = model_->WorldPose().Rot().X();
  spawn_call.request.initial_pose.orientation.y = model_->WorldPose().Rot().Y();
  spawn_call.request.initial_pose.orientation.z = model_->WorldPose().Rot().Z();
  spawn_call.request.model_name                 = "deployed_parachute";
  spawn_call.request.model_xml                  = parachute_model_xml;
  spawn_srv.call(spawn_call);
  if (!spawn_call.response.success) {
    ROS_ERROR("[%s][Parachute]: %s", uav_name.c_str(), spawn_call.response.status_message.c_str());
  }
  return spawn_call.response.success;
}
//}

/* deleteDeployedModel //{ */
bool ParachutePlugin::deleteDeployedModel() {
  ROS_INFO("[%s][Parachute]: Deleting deployed model", uav_name.c_str());
  gazebo_msgs::DeleteModel delete_call;
  delete_call.request.model_name = "deployed_parachute";
  delete_srv.call(delete_call);
  if (!delete_call.response.success) {
    ROS_ERROR("[%s][Parachute]: %s", uav_name.c_str(), delete_call.response.status_message.c_str());
  }
  return delete_call.response.success;
}
//}

GZ_REGISTER_MODEL_PLUGIN(ParachutePlugin)
}  // namespace gazebo
