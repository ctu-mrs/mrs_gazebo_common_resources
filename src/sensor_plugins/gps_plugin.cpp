/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
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

/**
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com> (original implementation)
 * @author Nuno Marques <nuno.marques@dronesolutions.io> (original implementation)
 * @author Vojtěch Spurný <vojtech.spurny@fel.cvut.cz> (rewrite and modify)
 * @author Robert Pěnička <robert.penicka@fel.cvut.cz> (rewrite and modify)
 */

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <ros/ros.h>

#include "common.h"
#include "SITLGps.pb.h"
#include "Groundtruth.pb.h"

#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/SonarSensor.hh>

// ROS Topic subscriber
#include <thread>
#include <mutex>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int64.h>

namespace gazebo
{
class GAZEBO_VISIBLE GpsPlugin : public ModelPlugin {
public:
  GpsPlugin();
  virtual ~GpsPlugin();

  void onGpsBlocking(const sensor_msgs::LaserScan::ConstPtr& _msg) {
    // ROS_INFO_STREAM("received LaserScan gps blocking");
    this->last_laser_message_mutex.lock();
    this->last_laser_message = *_msg;
    this->last_laser_message_mutex.unlock();
  }

  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);
  // void onGpsBlocking(sensor_msgs::LaserScan::ConstPtr& input);

private:
  std::pair<double, double> reproject(ignition::math::Vector3d& pos);

  std::string                     namespace_;
  std::default_random_engine      random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  physics::EntityPtr modelEntity;

  bool gps_noise_;
  bool gps_indoor_jamming_;

  physics::ModelPtr    model_;
  physics::WorldPtr    world_;
  event::ConnectionPtr updateConnection_;

  transport::NodePtr      node_handle_;
  transport::PublisherPtr gt_pub_;
  transport::PublisherPtr gps_pub_;

  boost::thread callback_queue_thread_;


  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber                  gps_block_sub_;
  ros::Publisher                   num_satelites_pub_;
  ros::CallbackQueue               rosQueue;
  std::thread                      rosQueueThread;

  sensor_msgs::msgs::SITLGps     gps_msg;
  sensor_msgs::msgs::Groundtruth groundtruth_msg;
  sensor_msgs::LaserScan         last_laser_message;
  std::mutex                     last_laser_message_mutex;

  common::Time last_gps_time_;
  common::Time last_time_;

  // Set global reference point
  // Zurich Irchel Park: 47.397742, 8.545594, 488m
  // Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
  // Moscow downtown: 55.753395, 37.625427, 155m

  // The home position can be specified using the environment variables:
  // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT

  // Zurich Irchel Park
  double lat_home = 47.397742 * M_PI / 180.0;  // rad
  double lon_home = 8.545594 * M_PI / 180.0;   // rad
  double alt_home = 488.0;                     // meters
  // Seattle downtown (15 deg declination): 47.592182, -122.316031
  // static const double lat_home = 47.592182 * M_PI / 180;    // rad
  // static const double lon_home = -122.316031 * M_PI / 180;  // rad
  // static const double alt_home = 86.0;                      // meters

  // Documentation inherited
  // protected: virtual bool UpdateImpl(const bool _force);
  // Documentation inherited
  // protected: virtual void Fini();
  // Documentation inherited
  // public: virtual void Init();
  // public: void OnUpdate();
  std::string modelName;
  // gazebo::sensor::SonarSensor sonarSensor;

  static constexpr const double earth_radius = 6353000.0;  // meters

  // gps delay related
  static constexpr double                gps_update_interval_ = 0.2;   // 5hz
  static constexpr double                gps_delay            = 0.12;  // 120 ms
  static constexpr int                   gps_buffer_size_max  = 1000;
  std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer;

  ignition::math::Vector3d gps_bias;
  ignition::math::Vector3d noise_gps_pos;
  ignition::math::Vector3d noise_gps_vel;
  ignition::math::Vector3d random_walk_gps;
  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  // gps noise parameters
  double                          std_xy;  // meters
  double                          std_z;   // meters
  std::default_random_engine      rand_;
  std::normal_distribution<float> randn_;
  static constexpr double         gps_corellation_time  = 60.0;  // s
  static constexpr double         gps_xy_random_walk    = 2.0;   // (m/s) / sqrt(hz)
  static constexpr double         gps_z_random_walk     = 4.0;   // (m/s) / sqrt(hz)
  static constexpr double         gps_xy_noise_density  = 2e-4;  // (m) / sqrt(hz)
  static constexpr double         gps_z_noise_density   = 4e-4;  // (m) / sqrt(hz)
  static constexpr double         gps_vxy_noise_density = 2e-1;  // (m/s) / sqrt(hz)
  static constexpr double         gps_vz_noise_density  = 4e-1;  // (m/s) / sqrt(hz)
};                                                               // class GAZEBO_VISIBLE GpsPlugin


GpsPlugin::GpsPlugin() : ModelPlugin() {
}

GpsPlugin::~GpsPlugin() {
  updateConnection_->~Connection();
}

void GpsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_     = world_->SimTime();
  last_gps_time_ = world_->SimTime();
#else
  last_time_                                  = world_->GetSimTime();
  last_gps_time_                              = world_->GetSimTime();
#endif

  // Use environment variables if set for home position.
  const char* env_lat = std::getenv("PX4_HOME_LAT");
  const char* env_lon = std::getenv("PX4_HOME_LON");
  const char* env_alt = std::getenv("PX4_HOME_ALT");

  // Get noise param
  if (_sdf->HasElement("gpsNoise")) {
    getSdfParam<bool>(_sdf, "gpsNoise", gps_noise_, gps_noise_);
  } else {
    gps_noise_ = false;
  }

  // Get noise param
  if (_sdf->HasElement("indoorJamming")) {
    getSdfParam<bool>(_sdf, "indoorJamming", gps_indoor_jamming_, gps_indoor_jamming_);
  } else {
    gps_indoor_jamming_ = false;
  }

  if (env_lat) {
    gzmsg << "Home latitude is set to " << env_lat << ".\n";
    lat_home = std::stod(env_lat) * M_PI / 180.0;
  }
  if (env_lon) {
    gzmsg << "Home longitude is set to " << env_lon << ".\n";
    lon_home = std::stod(env_lon) * M_PI / 180.0;
  }
  if (env_alt) {
    gzmsg << "Home altitude is set to " << env_alt << ".\n";
    alt_home = std::stod(env_alt);
  }

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  } else {
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GpsPlugin::OnUpdate, this, _1));

  gravity_W_ = world_->Gravity();

  this->modelName = model_->GetName();

  if (gps_indoor_jamming_) {
    // ROS Topic subscriber
    // Initialize ROS, if it has not already bee initialized.
    // ROS_INFO_STREAM("initialize ros in mrs gps plugin");
    if (!ros::isInitialized()) {
      int    argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "gazebo_ros_sub_gps", ros::init_options::NoSigintHandler);
      // ROS_INFO_STREAM("ros initialized");
    }

    // ROS_INFO_STREAM("create ros node");
    // Create our ROS node. This acts in a similar manner to the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client_gps"));

    std::string gps_blocking_topic = "/" + model_->GetName() + "/gps_sat_blocking";
    ROS_INFO_STREAM("want to subscribe " << gps_blocking_topic);
    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<sensor_msgs::LaserScan>(gps_blocking_topic, 1, boost::bind(&GpsPlugin::onGpsBlocking, this, _1),
                                                                                     ros::VoidPtr(), &this->rosQueue);

    // ROS_INFO_STREAM("created subscribe option so ");
    this->gps_block_sub_ = this->rosNode->subscribe(so);
    ROS_INFO_STREAM("gps plugin subscribed " << gps_blocking_topic);

    std::string gps_visible_sat_topic = "/" + model_->GetName() + "/gps_sat_num_visible";
    this->num_satelites_pub_          = this->rosNode->advertise<std_msgs::Int64>(gps_visible_sat_topic, 1);


    this->rosQueueThread = std::thread(std::bind(&GpsPlugin::QueueThread, this));
  }

  gps_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLGps>("~/" + model_->GetName() + "/gps", 10);
  gt_pub_  = node_handle_->Advertise<sensor_msgs::msgs::Groundtruth>("~/" + model_->GetName() + "/groundtruth", 10);
}

void GpsPlugin::OnUpdate(const common::UpdateInfo&) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time             current_time       = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = model_->WorldPose();  // TODO(burrimi): Check tf
#else
  ignition::math::Pose3d   T_W_I              = ignitionFromGazeboMath(model_->GetWorldPose());  // TODO(burrimi): Check tf
#endif
  ignition::math::Vector3d& pos_W_I = T_W_I.Pos();  // Use the models' world position for GPS and groundtruth

  // reproject position without noise into geographic coordinates
  auto latlon_gt = reproject(pos_W_I);

  // Use the models' world position for GPS velocity.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d velocity_current_W = model_->WorldLinearVel();
#else
  ignition::math::Vector3d velocity_current_W = ignitionFromGazeboMath(model_->GetWorldLinearVel());
#endif

  ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.Z()                      = 0;

  int numtelites = 0;
  if (gps_indoor_jamming_) {
    this->last_laser_message_mutex.lock();
    sensor_msgs::LaserScan laser_message = this->last_laser_message;
    this->last_laser_message_mutex.unlock();
    for (size_t i = 0; i < laser_message.ranges.size(); ++i) {

      if (std::isinf(laser_message.ranges[i])) {
        numtelites++;
      }
    }
    ROS_INFO_STREAM_THROTTLE(2, "gps on " << model_->GetName() << " see satelites " << numtelites << "/" << laser_message.ranges.size());
    std_msgs::Int64 num_sat_msg;
    num_sat_msg.data = numtelites;
    num_satelites_pub_.publish(num_sat_msg);
  }


  // update noise parameters if gps_noise_ is set
  if (gps_noise_) {
    noise_gps_pos.X()   = gps_xy_noise_density * sqrt(dt) * randn_(rand_);
    noise_gps_pos.Y()   = gps_xy_noise_density * sqrt(dt) * randn_(rand_);
    noise_gps_pos.Z()   = gps_z_noise_density * sqrt(dt) * randn_(rand_);
    noise_gps_vel.X()   = gps_vxy_noise_density * sqrt(dt) * randn_(rand_);
    noise_gps_vel.Y()   = gps_vxy_noise_density * sqrt(dt) * randn_(rand_);
    noise_gps_vel.Z()   = gps_vz_noise_density * sqrt(dt) * randn_(rand_);
    random_walk_gps.X() = gps_xy_random_walk * sqrt(dt) * randn_(rand_);
    random_walk_gps.Y() = gps_xy_random_walk * sqrt(dt) * randn_(rand_);
    random_walk_gps.Z() = gps_z_random_walk * sqrt(dt) * randn_(rand_);
  } else {
    noise_gps_pos.X()   = 0.0;
    noise_gps_pos.Y()   = 0.0;
    noise_gps_pos.Z()   = 0.0;
    noise_gps_vel.X()   = 0.0;
    noise_gps_vel.Y()   = 0.0;
    noise_gps_vel.Z()   = 0.0;
    random_walk_gps.X() = 0.0;
    random_walk_gps.Y() = 0.0;
    random_walk_gps.Z() = 0.0;
  }

  // gps bias integration
  gps_bias.X() += random_walk_gps.X() * dt - gps_bias.X() / gps_corellation_time;
  gps_bias.Y() += random_walk_gps.Y() * dt - gps_bias.Y() / gps_corellation_time;
  gps_bias.Z() += random_walk_gps.Z() * dt - gps_bias.Z() / gps_corellation_time;

  // reproject position with noise into geographic coordinates
  auto pos_with_noise = pos_W_I + noise_gps_pos + gps_bias;
  auto latlon         = reproject(pos_with_noise);

  // standard deviation TODO: add a way of computing this
  std_xy = 1.0;
  std_z  = 1.0;
  if (gps_indoor_jamming_) {
    // ROS_INFO_STREAM_THROTTLE(2, "jamming GPS of " << namespace_);
    // ROS_INFO_STREAM("jamming GPS of "<<namespace_);
    // gzmsg << "std_xy "<<std_xy<<" std_z " << std_z << "\n";
  }

  // fill SITLGps msg
  gps_msg.set_time(current_time.Double());
  gps_msg.set_latitude_deg(latlon.first * 180.0 / M_PI);
  gps_msg.set_longitude_deg(latlon.second * 180.0 / M_PI);
  gps_msg.set_altitude(pos_W_I.Z() + alt_home + noise_gps_pos.Z() + gps_bias.Z());
  gps_msg.set_eph(std_xy);
  gps_msg.set_epv(std_z);
  gps_msg.set_velocity(velocity_current_W_xy.Length());
  gps_msg.set_velocity_east(velocity_current_W.X() + noise_gps_vel.Y());
  gps_msg.set_velocity_north(velocity_current_W.Y() + noise_gps_vel.X());
  gps_msg.set_velocity_up(velocity_current_W.Z() + noise_gps_vel.Z());

  // ROS_INFO_STREAM_THROTTLE(0.2,"push gps_delay_buffer");
  // add msg to buffer
  gps_delay_buffer.push(gps_msg);

  // apply GPS delay
  if ((current_time - last_gps_time_).Double() > gps_update_interval_) {
    last_gps_time_ = current_time;

    while (true) {
      gps_msg                  = gps_delay_buffer.front();
      double gps_current_delay = current_time.Double() - gps_delay_buffer.front().time();
      if (gps_delay_buffer.empty()) {
        // abort if buffer is empty already
        break;
      }
      // remove data that is too old or if buffer size is too large
      if (gps_current_delay > gps_delay) {
        gps_delay_buffer.pop();
        // remove data if buffer too large
      } else if (gps_delay_buffer.size() > gps_buffer_size_max) {
        gps_delay_buffer.pop();
      } else {
        // if we get here, we have good data, stop
        break;
      }
    }
    // publish SITLGps msg at 5hz

    gps_pub_->Publish(gps_msg);
  }

  // ROS_INFO_STREAM_THROTTLE(0.2,"fill groundtruth_msg");
  // fill Groundtruth msg
  groundtruth_msg.set_time(current_time.Double());
  groundtruth_msg.set_latitude_rad(latlon_gt.first);
  groundtruth_msg.set_longitude_rad(latlon_gt.second);
  groundtruth_msg.set_altitude(-pos_W_I.Z() + alt_home);
  groundtruth_msg.set_velocity_east(velocity_current_W.X());
  groundtruth_msg.set_velocity_north(velocity_current_W.Y());
  groundtruth_msg.set_velocity_up(velocity_current_W.Z());

  // ROS_INFO_STREAM_THROTTLE(0.2,"Groundtruth publish");
  // publish Groundtruth msg at full rate
  gt_pub_->Publish(groundtruth_msg);

  last_time_ = current_time;
}

std::pair<double, double> GpsPlugin::reproject(ignition::math::Vector3d& pos) {
  // reproject local position to gps coordinates
  double x_rad = pos.Y() / earth_radius;  // north
  double y_rad = pos.X() / earth_radius;  // east
  double c     = sqrt(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);
  double lat_rad, lon_rad;

  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_home) + (x_rad * sin_c * cos(lat_home)) / c);
    lon_rad = (lon_home + atan2(y_rad * sin_c, c * cos(lat_home) * cos_c - x_rad * sin(lat_home) * sin_c));
  } else {
    lat_rad = lat_home;
    lon_rad = lon_home;
  }

  return std::make_pair(lat_rad, lon_rad);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GpsPlugin)
}  // namespace gazebo
