#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/physics/Model.hh>

#include <random>
#include <sdf/sdf.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <std_srvs/Trigger.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>

#include <ignition/math.hh>

#include <boost/thread.hpp>

namespace gazebo
{

/* Class definition */  //{
class WaterGunPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

private:
  physics::ModelPtr  model_;
  ros::NodeHandle *  nh_;
  ros::ServiceServer start_srv, stop_srv, cleanup_srv, fill_tank_srv;
  ros::ServiceClient spawn_srv, delete_srv, velocity_srv, gun_pose_srv;
  ros::Subscriber    odom_sub;

  bool startCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool stopCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool fillTank();
  bool cleanupCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  void updateParticleSpawnPoint();
  void OnUpdate();

  boost::thread run_gun_thread;
  void runGunThreadFunc();
  bool run = false;

  bool fillTank(int particles);

  std::string              particle_model;
  double                   muzzle_velocity   = 8.0;
  double                   spread            = 1.0;
  int                      particle_capacity = 20;
  int                      ammo              = 0;
  std::string              parent_name;
  std::string              spawning_reservoir;
  ignition::math::Vector3d offset;

  std::mt19937                           rng;
  std::uniform_real_distribution<double> rand_dbl;
  ignition::math::Pose3d                 spawn_point;

  event::ConnectionPtr updateConnection;
};
//}

/* Load */  //{
void WaterGunPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  model_      = _parent;
  parent_name = model_->GetName().c_str();

  std::stringstream particle_model_path;
  particle_model_path << ros::package::getPath("mrs_gazebo_common_resources") << "/models/water_particle/model.sdf";
  ROS_INFO("[%s][Water gun]: trying to open file %s", parent_name.c_str(), particle_model_path.str().c_str());
  try {
    particle_model = sdf::readFile(particle_model_path.str())->ToString();
    ROS_INFO("[%s][Water gun]: particle model loaded successfully", parent_name.c_str());
  }
  catch (...) {
    ROS_FATAL("[%s][Water gun]: cannot read particle model file!", parent_name.c_str());
  }

  if (_sdf->HasElement("spread")) {
    spread = _sdf->Get<double>("spread");
  } else {
    ROS_WARN("[%s][Water gun]: particle spread not defined! Used default value: 1.0", parent_name.c_str());
  }
  rand_dbl = std::uniform_real_distribution<double>(-spread / 2, spread / 2);

  if (_sdf->HasElement("muzzle_velocity")) {
    muzzle_velocity = _sdf->Get<double>("muzzle_velocity");
  } else {
    ROS_WARN("[%s][Water gun]: muzzle velocity not defined! Used default value: 8.0 m/s", parent_name.c_str());
  }
  if (_sdf->HasElement("particle_capacity")) {
    particle_capacity = _sdf->Get<int>("particle_capacity");
  } else {
    ROS_WARN("[%s][Water gun]: muzzle velocity not defined! Used default value: 20", parent_name.c_str());
  }

  if (_sdf->HasElement("spawning_reservoir")) {
    spawning_reservoir = _sdf->Get<std::string>("spawning_reservoir");
  } else {
    ROS_WARN("[%s][Water gun]: spawning reservoir not defined. Particles will spawn at [0 0 0]", parent_name.c_str());
  }

  if (_sdf->HasElement("offset_x")) {
    offset.X() = _sdf->Get<double>("offset_x");
  } else {
    ROS_WARN("[%s][Water gun]: Spawn offset_x not defined. New particles may strike the UAV", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_y")) {
    offset.Y() = _sdf->Get<double>("offset_y");
  } else {
    ROS_WARN("[%s][Water gun]: Spawn offset_y not defined. New particles may strike the UAV", parent_name.c_str());
  }
  if (_sdf->HasElement("offset_z")) {
    offset.Z() = _sdf->Get<double>("offset_z");
  } else {
    ROS_WARN("[%s][Water gun]: Spawn offset_z not defined. New particles may strike the UAV", parent_name.c_str());
  }

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  run = false;
  ros::init(argc, argv, "water_gun_plugin", ros::init_options::NoSigintHandler);
  nh_ = new ros::NodeHandle("~");

  // connect to gazebo model control services
  spawn_srv    = nh_->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  delete_srv   = nh_->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  velocity_srv = nh_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gun_pose_srv = nh_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/get_model_state");

  std::stringstream ss;

  updateParticleSpawnPoint();
  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WaterGunPlugin::OnUpdate, this));

  // create ROS services to control the water gun
  ss << "/" << parent_name << "/water_gun/start";
  start_srv = nh_->advertiseService(ss.str().c_str(), &WaterGunPlugin::startCallback, this);
  ss.str(std::string());
  ss << "/" << parent_name << "/water_gun/stop";
  stop_srv = nh_->advertiseService(ss.str().c_str(), &WaterGunPlugin::stopCallback, this);
  ss.str(std::string());
  ss << "/" << parent_name << "/water_gun/cleanup";
  /* cleanup_srv = nh_->advertiseService(ss.str().c_str(), &WaterGunPlugin::cleanupCallback, this); */
  ss.str(std::string());
  ss << "/" << parent_name << "/water_gun/fill_tank";
  /* fill_tank_srv = nh_->advertiseService(ss.str().c_str(), &WaterGunPlugin::fillTankCallback, this); */

  /* fillTank(); */
  run_gun_thread = boost::thread(boost::bind(&WaterGunPlugin::runGunThreadFunc, this));
  ROS_INFO("[%s][Water gun]: Initialized new water gun", parent_name.c_str());
}
//}

/* fillTank */  //{
bool WaterGunPlugin::fillTank() {
  ROS_INFO("[Water gun]: Filling tank with %d particles", particle_capacity);
  gazebo_msgs::SpawnModel spawn_call;
  spawn_call.request.reference_frame = spawning_reservoir;
  spawn_call.request.model_xml       = particle_model;

  for (int i = particle_capacity - 1; i >= 0; i--) {
    std::stringstream ss;
    ss << parent_name << "_water_particle_" << i;
    spawn_call.request.model_name = ss.str().c_str();

    spawn_srv.call(spawn_call);
    ammo++;
  }
  ROS_INFO("[%s][Water gun]: Tank full", parent_name.c_str());
  /* res.success = true; */
  return true;
}
//}

/* startCallback */  //{
bool WaterGunPlugin::startCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  ROS_INFO("[%s][Water gun]: Make it rain!", parent_name.c_str());

  run = true;
  res.success = true;
  return res.success;
}

//}

/* stopCallback */  //{
bool WaterGunPlugin::stopCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  ROS_INFO("[%s][Water gun]: Stop the rain!", parent_name.c_str());

  run = false;
  res.success = true;
  return res.success;
}

//}

/* runGunThreadFunc */  //{
void WaterGunPlugin::runGunThreadFunc() {
  /* ROS_INFO("[%s][Water gun]: runGunThreadFunc!", parent_name.c_str()); */
  ros::Duration(5.0).sleep();
  fillTank();
  while (true) {

    if (ammo > 0 && run){
      ignition::math::Vector3d velocity_vector;
      velocity_vector.X() = muzzle_velocity;
      velocity_vector.Y() = rand_dbl(rng);
      velocity_vector.Z() = rand_dbl(rng);
      velocity_vector     = spawn_point.Rot() * velocity_vector;
      std::stringstream ss;
      ss << parent_name << "_water_particle_" << --ammo;
      /* ROS_INFO("[%s][Water gun]: bum %d !", parent_name.c_str(),ammo); */
      gazebo_msgs::SetModelState velocity_call;
      velocity_call.request.model_state.model_name      = ss.str().c_str();
      velocity_call.request.model_state.twist.linear.x  = velocity_vector.X();
      velocity_call.request.model_state.twist.linear.y  = velocity_vector.Y();
      velocity_call.request.model_state.twist.linear.z  = velocity_vector.Z();
      velocity_call.request.model_state.pose.position.x = spawn_point.Pos().X();
      velocity_call.request.model_state.pose.position.y = spawn_point.Pos().Y();
      velocity_call.request.model_state.pose.position.z = spawn_point.Pos().Z();
      velocity_srv.call(velocity_call);
    }
    else if (ammo == 0)
      ammo = particle_capacity;
    ros::Duration(0.1).sleep();
  }
  run = false;
  return;
}

//}

/* cleanupCallback */  //{
bool WaterGunPlugin::cleanupCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  ROS_INFO("[%s][Water gun]: Removing existing particles", parent_name.c_str());

  int i = 0;
  while (1) {
    std::stringstream ss;
    ss << parent_name << "_water_particle_" << i;
    gazebo_msgs::DeleteModel delete_call;
    delete_call.request.model_name = ss.str().c_str();
    delete_srv.call(delete_call);
    if (!delete_call.response.success || i > particle_capacity) {
      /* ROS_INFO("[%s][Water gun]: Removed %d particles", parent_name.c_str(), i); */
      break;
    }
    i++;
  }
  res.success = true;
  return res.success;
}
//}

/* OnUpdate //{ */
void WaterGunPlugin::OnUpdate() {
  updateParticleSpawnPoint();
}
//}

/* updateParticleSpawnPoint //{ */
void WaterGunPlugin::updateParticleSpawnPoint() {
  spawn_point.Pos() = model_->WorldPose().Pos() + model_->WorldPose().Rot() * offset;
  spawn_point.Rot() = model_->WorldPose().Rot();
}
//}

GZ_REGISTER_MODEL_PLUGIN(WaterGunPlugin)
}  // namespace gazebo
