/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/** \author Nathan Koenig (original implementation)
    \author Vojtěch Spurný  - vojtech.spurny@fel.cvut.cz (rewrite and modify)
*/

#include <string>
#include <assert.h>
#include <algorithm>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
// --------------------------------------------------------------
// |               Class GazeboRosLaser definition              |
// --------------------------------------------------------------
class GazeboRosLaser : public RayPlugin {

public:
  /// \brief Constructor
  GazeboRosLaser();

  /// \brief Destructor
  ~GazeboRosLaser();

  /// \brief Load the plugin
  /// \param take in SDF root element
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  /// \brief Keep track of number of connctions
  int laser_connect_count_;

  void LaserConnect();

  void LaserDisconnect();

  // Pointer to the model
  GazeboRosPtr gazebo_ros_;

  std::string world_name_;

  physics::WorldPtr world_;

  /// \brief The parent sensor
  sensors::RaySensorPtr parent_ray_sensor_;

  /// \brief pointer to ros node
  ros::NodeHandle *rosnode_;

  ros::Publisher pub_;

  PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  std::string frame_name_;
  std::string parent_frame_name_;

  /// \brief frame transform parameters
  double x_, y_, z_, roll_, pitch_, yaw_;

  /// \brief for setting ROS name space
  std::string robot_namespace_;

  // deferred load in case ros is blocking
  sdf::ElementPtr sdf;

  void LoadThread();

  boost::thread deferred_load_thread_;

  unsigned int seed;

  gazebo::transport::NodePtr gazebo_node_;

  gazebo::transport::SubscriberPtr laser_scan_sub_;

  void OnScan(ConstLaserScanStampedPtr &_msg);

  /// \brief prevents blocking
  PubMultiQueue pmq;

  ros::Publisher      tf_pub_;
  tf2_msgs::TFMessage tf_message_;
  ros::WallTimer      timer_;
  void                publishStaticTransforms(const ros::WallTimerEvent &event);
  void                createStaticTransforms();
};

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosLaser::GazeboRosLaser() {
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosLaser::~GazeboRosLaser() {
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  // load plugin
  RayPlugin::Load(_parent, this->sdf);

  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_          = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("MRSGazebo2Dlirar plugin requires a Ray Sensor as its parent");

  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("frameName")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <frameName>, defaults to \"sensor\"");
    this->frame_name_ = "sensor";
  } else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("parentFrameName")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <parentFrameName>, defaults to \"world\"");
    this->parent_frame_name_ = "world";
  } else
    this->parent_frame_name_ = this->sdf->Get<std::string>("parentFrameName");

  if (!this->sdf->HasElement("x")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <x>, defaults to 0");
    this->x_ = 0;
  } else
    this->x_ = this->sdf->Get<double>("x");

  if (!this->sdf->HasElement("y")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <y>, defaults to 0");
    this->y_ = 0;
  } else
    this->y_ = this->sdf->Get<double>("y");

  if (!this->sdf->HasElement("z")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <z>, defaults to 0");
    this->z_ = 0;
  } else
    this->z_ = this->sdf->Get<double>("z");

  if (!this->sdf->HasElement("roll")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <roll>, defaults to 0");
    this->roll_ = 0;
  } else
    this->roll_ = this->sdf->Get<double>("roll");

  if (!this->sdf->HasElement("pitch")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <pitch>, defaults to 0");
    this->pitch_ = 0;
  } else
    this->pitch_ = this->sdf->Get<double>("pitch");

  if (!this->sdf->HasElement("yaw")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <yaw>, defaults to 0");
    this->yaw_ = 0;
  } else
    this->yaw_ = this->sdf->Get<double>("yaw");

  if (!this->sdf->HasElement("topicName")) {
    ROS_INFO_NAMED("2dlidar", "2dlidar plugin missing <topicName>, defaults to /2dlidar");
    this->topic_name_ = "/2dlidar";
  } else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  this->laser_connect_count_ = 0;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED("2dlidar", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  /* ROS_INFO_NAMED("2dlidar", "Starting Laser Plugin (ns = %s)", this->robot_namespace_.c_str()); */
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(boost::bind(&GazeboRosLaser::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::LoadThread() {
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  if (this->topic_name_ != "") {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(this->topic_name_, 1, boost::bind(&GazeboRosLaser::LaserConnect, this),
                                                                                     boost::bind(&GazeboRosLaser::LaserDisconnect, this), ros::VoidPtr(), NULL);
    this->pub_               = this->rosnode_->advertise(ao);
    this->pub_queue_         = this->pmq.addPub<sensor_msgs::LaserScan>();

    this->tf_pub_ = this->rosnode_->advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 100, true);

    createStaticTransforms();
    this->timer_ = this->rosnode_->createWallTimer(ros::WallDuration(1.0), &GazeboRosLaser::publishStaticTransforms, this);
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosLaser::createStaticTransforms() {
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp            = ros::Time::now();
  static_transformStamped.header.frame_id         = this->parent_frame_name_;
  static_transformStamped.child_frame_id          = this->frame_name_;
  static_transformStamped.transform.translation.x = this->x_;
  static_transformStamped.transform.translation.y = this->y_;
  static_transformStamped.transform.translation.z = this->z_;
  tf2::Quaternion quat;
  quat.setRPY(this->roll_, this->pitch_, this->yaw_);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  this->tf_message_.transforms.push_back(static_transformStamped);
  /* ROS_INFO_NAMED("2dlidar", "created"); */
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosLaser::publishStaticTransforms([[maybe_unused]] const ros::WallTimerEvent &event) {
  /* ROS_INFO("publishing"); */
  this->tf_pub_.publish(this->tf_message_);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosLaser::LaserConnect() {
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRosLaser::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosLaser::LaserDisconnect() {
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosLaser::OnScan(ConstLaserScanStampedPtr &_msg) {
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp    = ros::Time(_msg->time().sec(), _msg->time().nsec());
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min       = _msg->scan().angle_min();
  laser_msg.angle_max       = _msg->scan().angle_max();
  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment  = 0;  // instantaneous simulator scan
  laser_msg.scan_time       = 0;  // not sure whether this is correct
  laser_msg.range_min       = _msg->scan().range_min();
  laser_msg.range_max       = _msg->scan().range_max();
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(), _msg->scan().ranges().end(), laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(), _msg->scan().intensities().end(), laser_msg.intensities.begin());
  this->pub_queue_->push(laser_msg, this->pub_);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLaser)
}  // namespace gazebo
