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
#include <std_msgs/String.h>

#include <ignition/math.hh>
#include <iostream>
#include <fstream>

namespace gazebo
{

/* Class definition */  //{
class DynamicModelPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

private:
  physics::ModelPtr  model;
  ros::NodeHandle *  nh;
  ros::ServiceServer activation_srv, reset_srv;
  ros::Subscriber    load_map_sub;
  ros::Publisher     pose_pub;
  ros::Timer         motion_timer;

  void loadMapCallback(const std_msgs::String &msg);
  bool activationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  void   loadMap();
  void   segmentateMap();
  void   OnUpdate();
  void   motionTimerCallback(const ros::TimerEvent &event);
  void   moveModel();
  void   publishPose(ignition::math::Pose3d pose);
  double euclideanDistance(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2);

  ignition::math::Quaternion<double> convertEulerOffsetToQuaternion(double roll_offset, double pitch_offset, double yaw_offset);
  std::vector<double>    getSegmentationSteps(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2, std::vector<double> current_angles, int index_from);
  ignition::math::Pose3d getShiftedPose(ignition::math::Pose3d p, std::vector<double> xyzrpy_step);

  std::string            light_model;
  ignition::math::Pose3d spawn_point;
  std::string            trajectory_file;
  std::string            parent_name;
  double                 update_rate;
  uint                   trajectory_pointer;
  bool                   tracking_active;
  bool                   initial_on;
  bool                   loop_enabled;
  bool                   use_segmentation;
  bool                   use_directional_yaw;
  bool                   map_loaded  = false;
  bool                   initialized = false;


  std::vector<ignition::math::Pose3d> uploaded_trajectory;
  std::vector<ignition::math::Pose3d> segmented_trajectory;
  std::vector<double>                 velocities;
  std::vector<double>                 yaw_angles;
  std::vector<double>                 roll_angles;
  std::vector<double>                 pitch_angles;
  ignition::math::Pose3d              current_pose;
  ignition::math::Pose3d              previous_pose;
};
//}

/* Load */  //{
void DynamicModelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  ROS_INFO("[%s]: Dynamic model plugin started.", ros::this_node::getName().c_str());
  model       = _parent;
  parent_name = model->GetName().c_str();
  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->Get<double>("update_rate");
  } else {
    ROS_WARN("[%s][Dynamic model]: Update_rate not defined. Setting to defalt value.", parent_name.c_str());
    update_rate = 30;
  }
  if (_sdf->HasElement("initial_on")) {
    initial_on = _sdf->Get<bool>("initial_on");
  } else {
    ROS_WARN("[%s][Dynamic model]: Initial_on not defined. Setting to defalt value.", parent_name.c_str());
    initial_on = true;
  }
  if (_sdf->HasElement("loop_enabled")) {
    loop_enabled = _sdf->Get<bool>("loop_enabled");
  } else {
    ROS_WARN("[%s][Dynamic model]: Loop_enabled not defined. Setting to defalt value.", parent_name.c_str());
    loop_enabled = true;
  }
  if (_sdf->HasElement("trajectory_file")) {
    std::string path = ros::package::getPath("mrs_gazebo_common_resources");
    trajectory_file = path + "/models/dynamic_pickup/trajectories/"+_sdf->Get<std::string>("trajectory_file");
    ROS_INFO("[%s]: Trajectory file = %s ", ros::this_node::getName().c_str(), trajectory_file.c_str());
  } else {
    ROS_WARN("[%s][Dynamic model]: Map_path not defined. Has to be loaded by publishing on topic.", parent_name.c_str());
  }
  if (_sdf->HasElement("use_segmentation")) {
    use_segmentation = _sdf->Get<bool>("use_segmentation");
  } else {
    ROS_WARN("[%s][Dynamic model]: Use segmentation not defined. Setting to default value.", parent_name.c_str());
    use_segmentation = false;
  }
  if (_sdf->HasElement("use_directional_yaw")) {
    use_directional_yaw = _sdf->Get<bool>("use_directional_yaw");
  } else {
    ROS_WARN("[%s][Dynamic model]: Use directional yaw not defined. Setting to default value.", parent_name.c_str());
    use_directional_yaw = false;
  }

  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "dynamic_model_plugin", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle("~");

  // create ROS services to control the light
  std::stringstream ss;
  ss << "/gazebo/dynamic_model/" << parent_name << "/load_map";
  load_map_sub = nh->subscribe(ss.str().c_str(), 1, &DynamicModelPlugin::loadMapCallback, this);
  ss.str(std::string());
  ss << "/gazebo/dynamic_model/" << parent_name << "/activate";
  activation_srv = nh->advertiseService(ss.str().c_str(), &DynamicModelPlugin::activationCallback, this);
  ss.str(std::string());
  ss << "/gazebo/dynamic_model/" << parent_name << "/reset";
  reset_srv = nh->advertiseService(ss.str().c_str(), &DynamicModelPlugin::resetCallback, this);
  ss.str(std::string());
  ss << "/gazebo/dynamic_model/" << parent_name << "/odometry";
  motion_timer = nh->createTimer(ros::Rate(update_rate), &DynamicModelPlugin::motionTimerCallback, this);
  pose_pub     = nh->advertise<nav_msgs::Odometry>(ss.str().c_str(), 1);

  if (!trajectory_file.empty()) {
    loadMap();
  }

  initialized = true;
  ROS_INFO("[%s][Dynamic model]: Dynamic model plugin initialized.", parent_name.c_str());

  if (initial_on && map_loaded) {
    tracking_active = true;
    ROS_INFO("[%s]: Initial on flag set to true. Tracking activated.", ros::this_node::getName().c_str());
  }
}
//}

/* convertEulerOffsetToQuaternion() //{ */
ignition::math::Quaternion<double> DynamicModelPlugin::convertEulerOffsetToQuaternion(double roll_offset, double pitch_offset, double yaw_offset) {
  return ignition::math::Quaternion<double>(roll_offset, pitch_offset, yaw_offset);
}
//}

/* activationCallback() */  //{
bool DynamicModelPlugin::activationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Dynamic model]: Activation callback!", parent_name.c_str());
  res.success = true;
  if (req.data) {
    if (!map_loaded) {
      ROS_WARN("[%s]: Map not loaded yet. Tracking cannot be activated.", ros::this_node::getName().c_str());
      res.success = false;
      return false;
    }
    if (tracking_active) {
      ROS_WARN("[%s]: Tracking is already active.", ros::this_node::getName().c_str());
      res.success = false;
    } else {
      tracking_active = true;
      ROS_INFO("[%s]: Tracking activated.", ros::this_node::getName().c_str());
    }
  } else {
    if (tracking_active) {
      tracking_active = false;
      ROS_INFO("[%s]: Tracking deactivated", ros::this_node::getName().c_str());
    } else {
      ROS_WARN("[%s]: Tracking already deactivated", ros::this_node::getName().c_str());
      res.success = false;
    }
  }
  return res.success;
}
//}

/* resetCallback() */  //{
bool DynamicModelPlugin::resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  ROS_INFO("[%s][Dynamic model]: Reset callback!", parent_name.c_str());
  if (tracking_active) {
    ROS_WARN("[%s]: Tracking is active, cannot be reset.", ros::this_node::getName().c_str());
    res.success = false;
  } else {
    trajectory_pointer = 0;
    current_pose       = segmented_trajectory[trajectory_pointer];
    moveModel();
    ROS_INFO("[%s]: Tracking reset.", ros::this_node::getName().c_str());
    res.success = true;
  }
  return res.success;
}
//}

/* publishPose() //{ */

void DynamicModelPlugin::publishPose(ignition::math::Pose3d pose) {
  nav_msgs::Odometry odometry_msg;
  odometry_msg.pose.pose.position.x    = pose.Pos().X();
  odometry_msg.pose.pose.position.y    = pose.Pos().Y();
  odometry_msg.pose.pose.position.z    = pose.Pos().Z();
  odometry_msg.pose.pose.orientation.x = pose.Rot().X();
  odometry_msg.pose.pose.orientation.y = pose.Rot().Y();
  odometry_msg.pose.pose.orientation.z = pose.Rot().Z();
  odometry_msg.pose.pose.orientation.w = pose.Rot().W();
  odometry_msg.header.stamp       = ros::Time::now();
  odometry_msg.twist.twist.linear.x   = (pose.Pos().X() - previous_pose.Pos().X())*update_rate;
  odometry_msg.twist.twist.linear.y   = (pose.Pos().Y() - previous_pose.Pos().Y())*update_rate;
  odometry_msg.twist.twist.linear.z   = (pose.Pos().Z() - previous_pose.Pos().Z())*update_rate;
  previous_pose = pose;
  try {
    pose_pub.publish(odometry_msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pose_pub.getTopic().c_str());
  }
}

//}

/* loadMapCallback() */  //{
void DynamicModelPlugin::loadMapCallback(const std_msgs::String &msg) {
  ROS_INFO("[%s][DynamicModel]: Load map callback!", parent_name.c_str());
  trajectory_file = msg.data;
  loadMap();
}
//}

/* motionTimerCallback */  //{
void DynamicModelPlugin::motionTimerCallback(const ros::TimerEvent &event) {

  if (!initialized || !tracking_active) {
    return;
  }

  current_pose = segmented_trajectory[trajectory_pointer];
  moveModel();
  trajectory_pointer++;

  if (trajectory_pointer >= segmented_trajectory.size()) {
    if (loop_enabled) {
      trajectory_pointer = 0;
      ROS_INFO("[%s]: Last point of trajectory achieved. Loop enabled -> starting from the first point.", ros::this_node::getName().c_str());
    } else {
      tracking_active    = false;
      trajectory_pointer = 0;
      ROS_INFO("[%s]: Last point of trajectory achieved. Tracking deactivated.", ros::this_node::getName().c_str());
    }
  }

  publishPose(current_pose);
}
//}

/* euclideanDistance() */  //{
double DynamicModelPlugin::euclideanDistance(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2) {
  return sqrt(pow(p_1.Pos().X() - p_2.Pos().X(), 2) + pow(p_1.Pos().Y() - p_2.Pos().Y(), 2) + pow(p_1.Pos().Z() - p_2.Pos().Z(), 2));
}
//}

/* getShiftedPose() */  //{
ignition::math::Pose3d DynamicModelPlugin::getShiftedPose(ignition::math::Pose3d p, std::vector<double> xyzrpy_step) {
  ignition::math::Pose3d new_pose;
  new_pose.Pos().X() = p.Pos().X() + xyzrpy_step[0];
  new_pose.Pos().Y() = p.Pos().Y() + xyzrpy_step[1];
  new_pose.Pos().Z() = p.Pos().Z() + xyzrpy_step[2];
  new_pose.Rot()     = p.Rot() * convertEulerOffsetToQuaternion(xyzrpy_step[3], xyzrpy_step[4], xyzrpy_step[5]);
  return new_pose;
}
//}

/* loadMap */  //{
void DynamicModelPlugin::loadMap() {
  std::ifstream in(trajectory_file);
  ROS_INFO(" Loading MAP: ");
  double x, y, z, roll, pitch, yaw, v;
  uploaded_trajectory.clear();
  velocities.clear();
  roll_angles.clear();
  pitch_angles.clear();
  yaw_angles.clear();
  if (in.is_open()) {

    while (in >> x >> y >> z >> roll >> pitch >> yaw >> v) {
      ignition::math::Pose3d pose;
      pose.Pos().X() = x;
      pose.Pos().Y() = y;
      pose.Pos().Z() = z;
      pose.Rot()     = convertEulerOffsetToQuaternion(roll, pitch, yaw);
      roll_angles.push_back(fmod(roll + 2 * M_PI, 2 * M_PI));
      pitch_angles.push_back(fmod(pitch + 2 * M_PI, 2 * M_PI));
      yaw_angles.push_back(fmod(yaw + 2 * M_PI, 2 * M_PI));
      uploaded_trajectory.push_back(pose);
      velocities.push_back(v);
    }

    in.close();
    map_loaded = true;

    if (use_directional_yaw) {
      for (uint k = 0; k < yaw_angles.size(); k++) {
        yaw_angles[k]                = fmod(atan2(uploaded_trajectory[(k + 1) % yaw_angles.size()].Pos().Y() - uploaded_trajectory[k].Pos().Y(),
                                   uploaded_trajectory[(k + 1) % yaw_angles.size()].Pos().X() - uploaded_trajectory[k].Pos().X()) +
                                 2 * M_PI,
                             2 * M_PI);
        uploaded_trajectory[k].Rot() = convertEulerOffsetToQuaternion(roll_angles[k], pitch_angles[k], yaw_angles[k]);
      }
      yaw_angles[yaw_angles.size() - 1] = yaw_angles[0];
    }

    current_pose = uploaded_trajectory[0];
    ROS_INFO("[%s]: Map successfully loaded.", ros::this_node::getName().c_str());

    if (use_segmentation) {
      segmentateMap();
    } else {
      segmented_trajectory = uploaded_trajectory;
    }

  } else {
    ROS_ERROR("[%s]: Unable to open trajectory file.", ros::this_node::getName().c_str());
  }
}
//}

/* getSegmentationSteps() //{ */

std::vector<double> DynamicModelPlugin::getSegmentationSteps(ignition::math::Pose3d p_from, ignition::math::Pose3d p_to, std::vector<double> current_angles,
                                                             int idx_from) {
  std::vector<double> xyzrpy_steps;
  double              n_steps, dist;  // shouldnt be int!
  dist    = euclideanDistance(p_from, p_to);
  n_steps = dist / velocities[idx_from] * update_rate;
  ignition::math::Pose3d offset_step, last_pose;
  xyzrpy_steps.push_back((p_to.Pos().X() - p_from.Pos().X()) / n_steps);
  xyzrpy_steps.push_back((p_to.Pos().Y() - p_from.Pos().Y()) / n_steps);
  xyzrpy_steps.push_back((p_to.Pos().Z() - p_from.Pos().Z()) / n_steps);

  offset_step.Pos().X() = xyzrpy_steps[0];
  offset_step.Pos().Y() = xyzrpy_steps[1];
  offset_step.Pos().Z() = xyzrpy_steps[2];
  last_pose.Pos().X()   = p_from.Pos().X() + floor(n_steps) * xyzrpy_steps[0];
  last_pose.Pos().Y()   = p_from.Pos().Y() + floor(n_steps) * xyzrpy_steps[1];
  last_pose.Pos().Z()   = p_from.Pos().Z() + floor(n_steps) * xyzrpy_steps[2];

  if (euclideanDistance(last_pose, p_to) > euclideanDistance(last_pose + offset_step, p_to)) {
    n_steps = ceil(n_steps);
  } else {
    n_steps = floor(n_steps);
  }

  xyzrpy_steps.push_back((roll_angles[idx_from + 1] - current_angles[0]) / n_steps);
  xyzrpy_steps.push_back((pitch_angles[idx_from + 1] - current_angles[1]) / n_steps);
  xyzrpy_steps.push_back((yaw_angles[idx_from + 1] - current_angles[2]) / n_steps);
  xyzrpy_steps.push_back(n_steps);

  if (fabs(yaw_angles[idx_from + 1] - current_angles[2]) > M_PI) {
    xyzrpy_steps[5] = (2 * M_PI - fabs(yaw_angles[idx_from + 1] - current_angles[2])) / n_steps;
    xyzrpy_steps[5] = (yaw_angles[idx_from + 1] - current_angles[2]) > 0 ? -1 * xyzrpy_steps[5] : xyzrpy_steps[5];
  }

  return xyzrpy_steps;
}

//}

/* segmentateMape() //{ */

void DynamicModelPlugin::segmentateMap() {
  ignition::math::Pose3d current_pose, next_pose, step_pose_3d;
  double                 n_steps;
  std::vector<double>    xyzrpy_steps;
  std::vector<double>    current_angles;
  ignition::math::Pose3d current_pose_tmp;
  current_pose_tmp = uploaded_trajectory[0];
  current_angles.push_back(roll_angles[0]);
  current_angles.push_back(pitch_angles[0]);
  current_angles.push_back(yaw_angles[0]);
  std::vector<ignition::math::Pose3d> local_segmented_trajectory;

  for (uint i = 1; i < uploaded_trajectory.size(); i++) {
    xyzrpy_steps           = getSegmentationSteps(current_pose_tmp, uploaded_trajectory[i], current_angles, i - 1);
    n_steps                = xyzrpy_steps[6];
    step_pose_3d.Pos().X() = xyzrpy_steps[0];
    step_pose_3d.Pos().Y() = xyzrpy_steps[1];
    step_pose_3d.Pos().Z() = xyzrpy_steps[2];
    step_pose_3d.Rot()     = convertEulerOffsetToQuaternion(xyzrpy_steps[3], xyzrpy_steps[4], xyzrpy_steps[5]);
    for (int k = 0; k < n_steps; k++) {
      current_pose_tmp.Pos() += step_pose_3d.Pos();
      current_pose_tmp.Rot() = step_pose_3d.Rot() * current_pose_tmp.Rot();
      local_segmented_trajectory.push_back(current_pose_tmp);
    }
    current_angles[0] = fmod(roll_angles[i - 1] + n_steps * xyzrpy_steps[3] + 2 * M_PI, 2 * M_PI);
    current_angles[1] = fmod(pitch_angles[i - 1] + n_steps * xyzrpy_steps[4] + 2 * M_PI, 2 * M_PI);
    current_angles[2] = fmod(yaw_angles[i - 1] + n_steps * xyzrpy_steps[5] + 2 * M_PI, 2 * M_PI);
  }
  
  segmented_trajectory = local_segmented_trajectory;
  trajectory_pointer = 0;
  current_pose       = segmented_trajectory[trajectory_pointer];
  moveModel();

  ROS_INFO("[%s]: Trajectory segmented.", ros::this_node::getName().c_str());
}

//}

/* moveModel() //{ */
void DynamicModelPlugin::moveModel() {
  model->SetRelativePose(current_pose);
}
//}

GZ_REGISTER_MODEL_PLUGIN(DynamicModelPlugin)
}  // namespace gazebo
