/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013-2015, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jose Capriles, Bence Magyar. (original implementation)
    \author Vojtěch Spurný  - vojtech.spurny@fel.cvut.cz (rewrite and modify)
*/

#include <string>

#include <algorithm>
#include <assert.h>

#include "gazebo_plugins/gazebo_ros_utils.h"

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

// --------------------------------------------------------------
// |               Class GazeboRosRange definition              |
// --------------------------------------------------------------

class GazeboRosRange : public RayPlugin {

public:
  /// \brief Constructor
  GazeboRosRange();

  /// \brief Destructor
  ~GazeboRosRange();

  /// \brief Load the plugin
  /// \param take in SDF root element
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

protected:
  /// \brief Update the controller
  virtual void OnNewLaserScans();

private:
  /// \brief Put range data to the ROS topic
  void PutRangeData(common::Time &_updateTime);

  /// \brief Keep track of number of connctions
  int range_connect_count_;

  void RangeConnect();

  void RangeDisconnect();

  // Pointer to the model
  physics::WorldPtr world_;

  /// \brief The parent sensor
  sensors::SensorPtr parent_sensor_;

  sensors::RaySensorPtr parent_ray_sensor_;

  /// \brief pointer to ros node
  ros::NodeHandle *rosnode_;

  ros::Publisher pub_;

  /// \brief ros message
  sensor_msgs::Range range_msg_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  std::string frame_name_;
  std::string parent_frame_name_;

  /// \brief frame transform parameters
  double x_, y_, z_, roll_, pitch_, yaw_;

  /// \brief radiation type : ultrasound or infrared
  std::string radiation_;

  /// \brief sensor field of view
  double fov_;

  /// \brief Gaussian noise
  double gaussian_noise_;

  /// \brief Gaussian noise generator
  double GaussianKernel(double mu, double sigma);

  /// \brief mutex to lock access to fields that are used in message callbacks
  boost::mutex lock_;

  /// \brief hack to mimic hokuyo intensity cutoff of 100
  double hokuyo_min_intensity_;

  /// update rate of this sensor
  double update_rate_;

  double update_period_;

  common::Time last_update_time_;

  /// \brief for setting ROS name space
  std::string robot_namespace_;

  ros::CallbackQueue range_queue_;

  void RangeQueueThread();

  boost::thread callback_queue_thread_;

  // deferred load in case ros is blocking
  sdf::ElementPtr sdf;

  void LoadThread();

  boost::thread deferred_load_thread_;

  unsigned int seed;

  ros::Publisher      tf_pub_;
  tf2_msgs::TFMessage tf_message_;
  ros::WallTimer      timer_;
  void                publishStaticTransforms(const ros::WallTimerEvent &event);
  void                createStaticTransforms();
};

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosRange::GazeboRosRange() {
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosRange::~GazeboRosRange() {
  this->range_queue_.clear();
  this->range_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRange::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  // load plugin
  RayPlugin::Load(_parent, this->sdf);

  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_          = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  this->last_update_time_ = common::Time(0);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosRange controller requires a Ray Sensor as its parent");

  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("frameName")) {
    ROS_INFO_NAMED("range", "Range plugin missing <frameName>, defaults to \"sensor\"");
    this->frame_name_ = "sensor";
  } else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("parentFrameName")) {
    ROS_INFO_NAMED("range", "Range plugin missing <parentFrameName>, defaults to \"world\"");
    this->parent_frame_name_ = "world";
  } else
    this->parent_frame_name_ = this->sdf->Get<std::string>("parentFrameName");

  if (!this->sdf->HasElement("x")) {
    ROS_INFO_NAMED("range", "Range plugin missing <x>, defaults to 0");
    this->x_ = 0;
  } else
    this->x_ = this->sdf->Get<double>("x");

  if (!this->sdf->HasElement("y")) {
    ROS_INFO_NAMED("range", "Range plugin missing <y>, defaults to 0");
    this->y_ = 0;
  } else
    this->y_ = this->sdf->Get<double>("y");

  if (!this->sdf->HasElement("z")) {
    ROS_INFO_NAMED("range", "Range plugin missing <z>, defaults to 0");
    this->z_ = 0;
  } else
    this->z_ = this->sdf->Get<double>("z");

  if (!this->sdf->HasElement("roll")) {
    ROS_INFO_NAMED("range", "Range plugin missing <roll>, defaults to 0");
    this->roll_ = 0;
  } else
    this->roll_ = this->sdf->Get<double>("roll");

  if (!this->sdf->HasElement("pitch")) {
    ROS_INFO_NAMED("range", "Range plugin missing <pitch>, defaults to 0");
    this->pitch_ = 0;
  } else
    this->pitch_ = this->sdf->Get<double>("pitch");

  if (!this->sdf->HasElement("yaw")) {
    ROS_INFO_NAMED("range", "Range plugin missing <yaw>, defaults to 0");
    this->yaw_ = 0;
  } else
    this->yaw_ = this->sdf->Get<double>("yaw");

  if (!this->sdf->HasElement("topicName")) {
    ROS_INFO_NAMED("range", "Range plugin missing <topicName>, defaults to /range");
    this->topic_name_ = "/range";
  } else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("radiation")) {
    ROS_WARN_NAMED("range", "Range plugin missing <radiation>, defaults to ultrasound");
    this->radiation_ = "ultrasound";

  } else
    this->radiation_ = _sdf->GetElement("radiation")->Get<std::string>();

  if (!this->sdf->HasElement("fov")) {
    ROS_WARN_NAMED("range", "Range plugin missing <fov>, defaults to 0.05");
    this->fov_ = 0.05;
  } else
    this->fov_ = _sdf->GetElement("fov")->Get<double>();

  if (!this->sdf->HasElement("gaussianNoise")) {
    ROS_INFO_NAMED("range", "Range plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  } else
    this->gaussian_noise_ = this->sdf->Get<double>("gaussianNoise");

  if (!this->sdf->HasElement("updateRate")) {
    ROS_INFO_NAMED("range", "Range plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  } else
    this->update_rate_ = this->sdf->Get<double>("updateRate");

  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;

  this->range_connect_count_ = 0;

  this->range_msg_.header.frame_id = this->frame_name_;
  if (this->radiation_ == std::string("ultrasound"))
    this->range_msg_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  else
    this->range_msg_.radiation_type = sensor_msgs::Range::INFRARED;

  this->range_msg_.field_of_view = fov_;
  this->range_msg_.max_range     = this->parent_ray_sensor_->RangeMax();
  this->range_msg_.min_range     = this->parent_ray_sensor_->RangeMin();

  // Init ROS
  if (ros::isInitialized()) {
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(boost::bind(&GazeboRosRange::LoadThread, this));
  } else {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRange::LoadThread() {
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  if (this->topic_name_ != "") {
    ros::AdvertiseOptions ao =
        ros::AdvertiseOptions::create<sensor_msgs::Range>(this->topic_name_, 1, boost::bind(&GazeboRosRange::RangeConnect, this),
                                                          boost::bind(&GazeboRosRange::RangeDisconnect, this), ros::VoidPtr(), &this->range_queue_);
    this->pub_ = this->rosnode_->advertise(ao);

    this->tf_pub_ = this->rosnode_->advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 100, true);

    createStaticTransforms();
    this->timer_ = this->rosnode_->createWallTimer(ros::WallDuration(1.0), &GazeboRosRange::publishStaticTransforms, this);
  }


  // Initialize the controller
  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for range
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosRange::RangeQueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosRange::createStaticTransforms() {
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
void GazeboRosRange::publishStaticTransforms([[maybe_unused]] const ros::WallTimerEvent &event) {
  /* ROS_INFO("publishing"); */
  this->tf_pub_.publish(this->tf_message_);
}
////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosRange::RangeConnect() {
  this->range_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosRange::RangeDisconnect() {
  this->range_connect_count_--;

  if (this->range_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the plugin
void GazeboRosRange::OnNewLaserScans() {
  if (this->topic_name_ != "") {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time cur_time = this->world_->SimTime();
#else
    common::Time cur_time = this->world_->GetSimTime();
#endif
    if (cur_time < this->last_update_time_) {
      ROS_WARN_NAMED("range", "Negative sensor update time difference detected.");
      this->last_update_time_ = cur_time;
    }

    if (cur_time - this->last_update_time_ >= this->update_period_) {
      common::Time sensor_update_time = this->parent_sensor_->LastUpdateTime();
      this->PutRangeData(sensor_update_time);
      this->last_update_time_ = cur_time;
    }
  } else {
    ROS_INFO_NAMED("range", "gazebo_ros_range topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put range data to the interface
void GazeboRosRange::PutRangeData(common::Time &_updateTime) {
  this->parent_ray_sensor_->SetActive(false);
  // --------------------------------------------------------------
  // |                 point scan from ray sensor                 |
  // --------------------------------------------------------------
  {
    boost::mutex::scoped_lock lock(this->lock_);
    // Add Frame Name
    this->range_msg_.header.frame_id   = this->frame_name_;
    this->range_msg_.header.stamp.sec  = _updateTime.sec;
    this->range_msg_.header.stamp.nsec = _updateTime.nsec;

    // find ray with minimal range
    range_msg_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();

    int num_ranges = parent_ray_sensor_->LaserShape()->GetSampleCount() * parent_ray_sensor_->LaserShape()->GetVerticalSampleCount();

    for (int i = 0; i < num_ranges; ++i) {
      double ray = parent_ray_sensor_->LaserShape()->GetRange(i);
      if (ray < range_msg_.range)
        range_msg_.range = ray;
    }

    // add Gaussian noise and limit to min/max range
    range_msg_.range += this->GaussianKernel(0, gaussian_noise_);

    // fill correctly range if it is out of range
    if (range_msg_.range < this->range_msg_.min_range) {
      range_msg_.range = -std::numeric_limits<double>::infinity();
    } else if (range_msg_.range > this->range_msg_.max_range) {
      range_msg_.range = -std::numeric_limits<double>::infinity();
    }

    this->parent_ray_sensor_->SetActive(true);

    // send data out via ros message
    if (this->range_connect_count_ > 0 && this->topic_name_ != "")
      this->pub_.publish(this->range_msg_);
  }
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosRange::GaussianKernel(double mu, double sigma) {
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put range data to the interface
void GazeboRosRange::RangeQueueThread() {
  static const double timeout = 0.01;

  while (this->rosnode_->ok()) {
    this->range_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRange)
}  // namespace gazebo
