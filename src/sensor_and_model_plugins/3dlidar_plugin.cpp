/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
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
 *   * Neither the name of Dataspeed Inc. nor the names of its
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
 *********************************************************************/

// Use the same source code for CPU and GPU plugins
#ifndef GAZEBO_GPU_RAY
#define GAZEBO_GPU_RAY 0
#endif

/* includes //{ */

// Custom Callback Queue
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <gazebo/common/Events.hh>
#include <gazebo_plugins/PubQueue.h>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/plugins/GpuRayPlugin.hh>
#else
#include <gazebo/plugins/RayPlugin.hh>
#endif
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#if GAZEBO_GPU_RAY
#define GazeboRos3DLaser GazeboRos3DGpuLaser
#define RayPlugin GpuRayPlugin
#define RaySensorPtr GpuRaySensorPtr
#endif

#include <tf2_msgs/TFMessage.h>

#include <algorithm>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/sensors/GpuRaySensor.hh>
#else
#include <gazebo/sensors/RaySensor.hh>
#endif
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

//}

/* defines //{ */

#if GAZEBO_GPU_RAY
#define RaySensor GpuRaySensor
#define STR_Gpu "Gpu"
#define STR_GPU_ "GPU "
#else
#define STR_Gpu ""
#define STR_GPU_ ""
#endif

//}

namespace gazebo
{

  class GazeboRos3DLaser : public RayPlugin
  {
  public:
    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    /* Constructor //{ */

    GazeboRos3DLaser()
    {
    }

    //}

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    /* Destructor //{ */

    ~GazeboRos3DLaser()
    {
      ////////////////////////////////////////////////////////////////////////////////
      // Finalize the controller / Custom Callback Queue
      laser_queue_.clear();
      laser_queue_.disable();
      callback_laser_queue_thread_.join();
    }

    //}

    ////////////////////////////////////////////////////////////////////////////////
    // Load the controller
    /* Load() //{ */

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      // Load plugin
      RayPlugin::Load(_parent, _sdf);

      // Initialize Gazebo node
      gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
      gazebo_node_->Init();

      GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
      // Get the parent ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
      parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
      parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
      if (!parent_ray_sensor_)
      {
        gzthrow("GazeboRos3D" << STR_Gpu << "Laser controller requires a " << STR_Gpu << "Ray Sensor as its parent");
      }

      robot_namespace_ = "/";
      if (_sdf->HasElement("robotNamespace"))
      {
        robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
      }

      if (!_sdf->HasElement("min_range"))
      {
        ROS_INFO("3D laser plugin missing <min_range>, defaults to 0");
        min_range_ = 0;
      } else
      {
        min_range_ = _sdf->GetElement("min_range")->Get<double>();
      }

      if (!_sdf->HasElement("max_range"))
      {
        ROS_INFO("3D laser plugin missing <max_range>, defaults to infinity");
        max_range_ = INFINITY;
      } else
      {
        max_range_ = _sdf->GetElement("max_range")->Get<double>();
      }

      min_intensity_ = std::numeric_limits<double>::lowest();
      if (!_sdf->HasElement("min_intensity"))
      {
        ROS_INFO("3D laser plugin missing <min_intensity>, defaults to no clipping");
      } else
      {
        min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();
      }

      if (!_sdf->HasElement("ordered"))
      {
        ROS_INFO("3D laser plugin missing <ordered>, defaults to false");
        ordered_ = false;
      } else
      {
        ordered_ = _sdf->GetElement("ordered")->Get<bool>();
      }

      if (!_sdf->HasElement("topicName"))
      {
        ROS_INFO("3D laser plugin missing <topicName>, defaults to /points");
        topic_name_ = "/points";
      } else
      {
        topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
      }

      if (!_sdf->HasElement("topicDiagName"))
      {
        ROS_INFO("3D laser plugin missing <topicDiagName>, defaults to /laser_diag");
        topic_diag_name_ = "/laser_diag";
      } else
      {
        topic_diag_name_ = _sdf->GetElement("topicDiagName")->Get<std::string>();
      }

      if (!_sdf->HasElement("gaussianNoise"))
      {
        ROS_INFO("3D laser plugin missing <gaussianNoise>, defaults to 0.0");
        gaussian_noise_ = 0;
      } else
      {
        gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
      }

      if (!_sdf->HasElement("parentFrameName"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <parentFrameName>, defaults to \"fcu\"");
        this->parent_frame_name_ = "fcu";
      } else
        this->parent_frame_name_ = _sdf->Get<std::string>("parentFrameName");

      if (!_sdf->HasElement("sensorFrameName"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <sensorFrameName>, defaults to \"sensor\"");
        this->sensor_frame_name_ = "sensor";
      } else
        this->sensor_frame_name_ = _sdf->Get<std::string>("sensorFrameName");

      if (!_sdf->HasElement("sensor_x"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <sensor_x>, defaults to 0");
        this->sensor_x_ = 0;
      } else
        this->sensor_x_ = _sdf->Get<double>("sensor_x");

      if (!_sdf->HasElement("sensor_y"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <sensor_y>, defaults to 0");
        this->sensor_y_ = 0;
      } else
        this->sensor_y_ = _sdf->Get<double>("sensor_y");

      if (!_sdf->HasElement("sensor_z"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <sensor_z>, defaults to 0");
        this->sensor_z_ = 0;
      } else
        this->sensor_z_ = _sdf->Get<double>("sensor_z");

      if (!_sdf->HasElement("sensor_roll"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <sensor_roll>, defaults to 0");
        this->sensor_roll_ = 0;
      } else
        this->sensor_roll_ = _sdf->Get<double>("sensor_roll");

      if (!_sdf->HasElement("sensor_pitch"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <sensor_pitch>, defaults to 0");
        this->sensor_pitch_ = 0;
      } else
        this->sensor_pitch_ = _sdf->Get<double>("sensor_pitch");

      if (!_sdf->HasElement("sensor_yaw"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <sensor_yaw>, defaults to 0");
        this->sensor_yaw_ = 0;
      } else
        this->sensor_yaw_ = _sdf->Get<double>("sensor_yaw");

      if (!_sdf->HasElement("lidarFrameName"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <lidarFrameName>, defaults to \"lidar\"");
        this->lidar_frame_name_ = "lidar";
      } else
        this->lidar_frame_name_ = _sdf->Get<std::string>("lidarFrameName");

      if (!_sdf->HasElement("lidar_x"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <lidar_x>, defaults to 0");
        this->lidar_x_ = 0;
      } else
        this->lidar_x_ = _sdf->Get<double>("lidar_x");

      if (!_sdf->HasElement("lidar_y"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <lidar_y>, defaults to 0");
        this->lidar_y_ = 0;
      } else
        this->lidar_y_ = _sdf->Get<double>("lidar_y");

      if (!_sdf->HasElement("lidar_z"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <lidar_z>, defaults to 0");
        this->lidar_z_ = 0;
      } else
        this->lidar_z_ = _sdf->Get<double>("lidar_z");

      if (!_sdf->HasElement("lidar_roll"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <lidar_roll>, defaults to 0");
        this->lidar_roll_ = 0;
      } else
        this->lidar_roll_ = _sdf->Get<double>("lidar_roll");

      if (!_sdf->HasElement("lidar_pitch"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <lidar_pitch>, defaults to 0");
        this->lidar_pitch_ = 0;
      } else
        this->lidar_pitch_ = _sdf->Get<double>("lidar_pitch");

      if (!_sdf->HasElement("lidar_yaw"))
      {
        ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <lidar_yaw>, defaults to 0");
        this->lidar_yaw_ = 0;
      } else
        this->lidar_yaw_ = _sdf->Get<double>("lidar_yaw");

      if (!_sdf->HasElement("imu"))
      {
        ROS_INFO("3D laser plugin missing <imu>, defaults to false");
      } else
        this->imu_ = _sdf->Get<bool>("imu");

      if (this->imu_)
      {
        if (!_sdf->HasElement("imuFrameName"))
        {
          ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <imuFrameName>, defaults to \"imu\"");
          this->imu_frame_name_ = "imu";
        } else
          this->imu_frame_name_ = _sdf->Get<std::string>("imuFrameName");

        if (!_sdf->HasElement("imu_x"))
        {
          ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <imu_x>, defaults to 0");
          this->imu_x_ = 0;
        } else
          this->imu_x_ = _sdf->Get<double>("imu_x");

        if (!_sdf->HasElement("imu_y"))
        {
          ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <imu_y>, defaults to 0");
          this->imu_y_ = 0;
        } else
          this->imu_y_ = _sdf->Get<double>("imu_y");

        if (!_sdf->HasElement("imu_z"))
        {
          ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <imu_z>, defaults to 0");
          this->imu_z_ = 0;
        } else
          this->imu_z_ = _sdf->Get<double>("imu_z");

        if (!_sdf->HasElement("imu_roll"))
        {
          ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <imu_roll>, defaults to 0");
          this->imu_roll_ = 0;
        } else
          this->imu_roll_ = _sdf->Get<double>("imu_roll");

        if (!_sdf->HasElement("imu_pitch"))
        {
          ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <imu_pitch>, defaults to 0");
          this->imu_pitch_ = 0;
        } else
          this->imu_pitch_ = _sdf->Get<double>("imu_pitch");

        if (!_sdf->HasElement("imu_yaw"))
        {
          ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin missing <imu_yaw>, defaults to 0");
          this->imu_yaw_ = 0;
        } else
          this->imu_yaw_ = _sdf->Get<double>("imu_yaw");
      }

      // Pre-fill the coordinate coefficients for the different rays
      /*  //{ */

      {
#if GAZEBO_MAJOR_VERSION >= 7
        const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
        const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

        const int rangeCount = parent_ray_sensor_->RangeCount();
        const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();

        const ignition::math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
        const ignition::math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();
#else
        math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
        math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

        const int rangeCount = parent_ray_sensor_->GetRangeCount();
        const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

        const math::Angle verticalMaxAngle = parent_ray_sensor_->GetVerticalAngleMax();
        const math::Angle verticalMinAngle = parent_ray_sensor_->GetVerticalAngleMin();
#endif

        const double yDiff = maxAngle.Radian() - minAngle.Radian();
        const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

        double yAngle_step;
        if (rangeCount > 1)
          yAngle_step = yDiff / (rangeCount - 1);
        else
          yAngle_step = 0;

        double pAngle_step;
        if (verticalRangeCount > 1)
          pAngle_step = pDiff / (verticalRangeCount - 1);
        else
          pAngle_step = 0;

        coord_coeffs_.reserve(verticalRangeCount * rangeCount);
        for (int i = 0; i < rangeCount; i++)
        {
          for (int j = 0; j < verticalRangeCount; j++)
          {
            // Get angles of ray to get xyz for point
            const double yAngle = i * yAngle_step + minAngle.Radian();
            const double pAngle = j * pAngle_step + verticalMinAngle.Radian();

            const double x_coeff = cos(pAngle) * cos(yAngle);
            const double y_coeff = cos(pAngle) * sin(yAngle);
#if GAZEBO_MAJOR_VERSION > 2
            const double z_coeff = sin(pAngle);
#else
            const double z_coeff = -sin(pAngle);
#endif
            coord_coeffs_.push_back({x_coeff, y_coeff, z_coeff});
          }
        }
      }

      //}

      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      // Create node handle
      nh_ = ros::NodeHandle(robot_namespace_);
      m_processing_scan = false;

      // Advertise publisher with a custom callback queue
      if (topic_name_ != "")
      {
        ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            topic_name_, 1, boost::bind(&GazeboRos3DLaser::ConnectCb, this), boost::bind(&GazeboRos3DLaser::ConnectCb, this), ros::VoidPtr(), &laser_queue_);
        pub_ = nh_.advertise(ao);
      }

      if (topic_diag_name_ != "")
      {
        pub_diag_ = nh_.advertise<std_msgs::Empty>(topic_diag_name_, 1);
      }

      // Sensor generation off by default
      parent_ray_sensor_->SetActive(false);

      // Start custom queue for laser
      callback_laser_queue_thread_ = boost::thread(boost::bind(&GazeboRos3DLaser::laserQueueThread, this));

      this->load_thread_ = boost::thread(boost::bind(&GazeboRos3DLaser::TransformThread, this));

#if GAZEBO_MAJOR_VERSION >= 7
      ROS_INFO("3D %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->VerticalRangeCount());
#else
      ROS_INFO("3D %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->GetVerticalRangeCount());
#endif
    }

    //}

  private:
    ////////////////////////////////////////////////////////////////////////////////
    /* TransformThread() //{ */

    void TransformThread()
    {
      /* while (!this->initialized_) { */
      /*   ros::Duration(0.01).sleep(); */
      /* } */
      ROS_INFO_NAMED("3d_lidar", "3d_lidar plugin - GazeboRos3DLaser initialized");
      this->tf_pub_ = this->nh_.advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 10, false);

      createStaticTransforms();
      this->timer_ = this->nh_.createWallTimer(ros::WallDuration(1.0), &GazeboRos3DLaser::publishStaticTransforms, this);
    }

    //}

    ////////////////////////////////////////////////////////////////////////////////
    /* createStaticTransforms() //{ */

    void createStaticTransforms()
    {
      geometry_msgs::TransformStamped static_transform_sensor_base;
      geometry_msgs::TransformStamped static_transform_lidar_sensor;
      geometry_msgs::TransformStamped static_transform_imu_sensor;

      tf2::Quaternion quat;
      ros::Time stamp = ros::Time::now();

      static_transform_sensor_base.header.stamp = stamp;
      static_transform_sensor_base.header.frame_id = this->parent_frame_name_;
      static_transform_sensor_base.child_frame_id = this->sensor_frame_name_;
      static_transform_sensor_base.transform.translation.x = this->sensor_x_;
      static_transform_sensor_base.transform.translation.y = this->sensor_y_;
      static_transform_sensor_base.transform.translation.z = this->sensor_z_;
      quat.setRPY(this->sensor_roll_, this->sensor_pitch_, this->sensor_yaw_);
      static_transform_sensor_base.transform.rotation.x = quat.x();
      static_transform_sensor_base.transform.rotation.y = quat.y();
      static_transform_sensor_base.transform.rotation.z = quat.z();
      static_transform_sensor_base.transform.rotation.w = quat.w();

      this->tf_message_.transforms.push_back(static_transform_sensor_base);

      static_transform_lidar_sensor.header.stamp = stamp;
      static_transform_lidar_sensor.header.frame_id = this->sensor_frame_name_;
      static_transform_lidar_sensor.child_frame_id = this->lidar_frame_name_;
      static_transform_lidar_sensor.transform.translation.x = this->lidar_x_;
      static_transform_lidar_sensor.transform.translation.y = this->lidar_y_;
      static_transform_lidar_sensor.transform.translation.z = this->lidar_z_;
      /* static_transform_lidar_sensor.transform.translation.z = 0.03618; */
      quat.setRPY(this->lidar_roll_, this->lidar_pitch_, this->lidar_yaw_);
      /* quat.setRPY(0, 0, M_PI); */
      static_transform_lidar_sensor.transform.rotation.x = quat.x();
      static_transform_lidar_sensor.transform.rotation.y = quat.y();
      static_transform_lidar_sensor.transform.rotation.z = quat.z();
      static_transform_lidar_sensor.transform.rotation.w = quat.w();

      this->tf_message_.transforms.push_back(static_transform_lidar_sensor);

      if (this->imu_)
      {
        static_transform_imu_sensor.header.stamp = stamp;
        static_transform_imu_sensor.header.frame_id = this->sensor_frame_name_;
        static_transform_imu_sensor.child_frame_id = this->imu_frame_name_;
        static_transform_imu_sensor.transform.translation.x = this->imu_x_;
        static_transform_imu_sensor.transform.translation.x = this->imu_y_;
        static_transform_imu_sensor.transform.translation.x = this->imu_z_;
        quat.setRPY(this->imu_roll_, this->imu_pitch_, this->imu_yaw_);
        /* static_transform_imu_sensor.transform.translation.x = 0.006253; */
        /* static_transform_imu_sensor.transform.translation.y = -0.011775; */
        /* static_transform_imu_sensor.transform.translation.z = 0.007645; */
        /* quat.setRPY(0, 0, 0); */
        static_transform_imu_sensor.transform.rotation.x = quat.x();
        static_transform_imu_sensor.transform.rotation.y = quat.y();
        static_transform_imu_sensor.transform.rotation.z = quat.z();
        static_transform_imu_sensor.transform.rotation.w = quat.w();

        this->tf_message_.transforms.push_back(static_transform_imu_sensor);
      }

      /* ROS_INFO_NAMED("camera", "created"); */
    }

    //}

    ////////////////////////////////////////////////////////////////////////////////
    /* publishStaticTransforms() //{ */

    void publishStaticTransforms([[maybe_unused]] const ros::WallTimerEvent& event)
    {
      /* ROS_INFO("publishing"); */
      this->tf_pub_.publish(this->tf_message_);
    }

    //}

    ////////////////////////////////////////////////////////////////////////////////
    // Subscribe on-demand
    /* ConnectCb() //{ */

    void ConnectCb()
    {
      boost::lock_guard<boost::mutex> lock(lock_);
      if (pub_.getNumSubscribers())
      {
        if (!sub_)
        {
#if GAZEBO_MAJOR_VERSION >= 7
          sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRos3DLaser::OnScan, this);
#else
          sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(), &GazeboRos3DLaser::OnScan, this);
#endif
        }
        parent_ray_sensor_->SetActive(true);
      } else
      {
#if GAZEBO_MAJOR_VERSION >= 7
        if (sub_)
        {
          sub_->Unsubscribe();
          sub_.reset();
        }
#endif
        parent_ray_sensor_->SetActive(false);
      }
    }

    //}

    /* OnScan() //{ */

    std::atomic_bool m_processing_scan;

    void OnScan(const ConstLaserScanStampedPtr& msg)
    {
      // make sure that only one scan is being processed at a time to avoid clogging the CPU
      // (rather throttle the scans)
      if (!m_processing_scan)
      {
        m_processing_scan = true;
        std::thread t(&GazeboRos3DLaser::processScan, this, msg);
        t.detach();
      }
    }

    void processScan(const ConstLaserScanStampedPtr& _msg)
    {
#if GAZEBO_MAJOR_VERSION >= 7
      const double maxRange = parent_ray_sensor_->RangeMax();
      const double minRange = parent_ray_sensor_->RangeMin();

      const int rangeCount = parent_ray_sensor_->RangeCount();
      const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();
#else
      const double maxRange = parent_ray_sensor_->GetRangeMax();
      const double minRange = parent_ray_sensor_->GetRangeMin();

      const int rangeCount = parent_ray_sensor_->GetRangeCount();
      const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();
#endif

      const double MIN_RANGE = std::max(min_range_, minRange);
      const double MAX_RANGE = std::min(max_range_, maxRange);
      const double MIN_INTENSITY = min_intensity_;

      // Populate message fields
      const uint32_t POINT_STEP = 32;
      sensor_msgs::PointCloud2 msg;
      msg.header.frame_id = lidar_frame_name_;
      msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
      msg.fields.resize(6);
      msg.fields[0].name = "x";
      msg.fields[0].offset = 0;
      msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
      msg.fields[0].count = 1;
      msg.fields[1].name = "y";
      msg.fields[1].offset = 4;
      msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
      msg.fields[1].count = 1;
      msg.fields[2].name = "z";
      msg.fields[2].offset = 8;
      msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      msg.fields[2].count = 1;
      msg.fields[3].name = "intensity";
      msg.fields[3].offset = 16;
      msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
      msg.fields[3].count = 1;
      msg.fields[4].name = "ring";
      msg.fields[4].offset = 20;
      msg.fields[4].datatype = sensor_msgs::PointField::UINT8;
      msg.fields[4].count = 1;
      msg.fields[5].name = "range";
      msg.fields[5].offset = 24;
      msg.fields[5].datatype = sensor_msgs::PointField::UINT32;
      msg.fields[5].count = 1;
      msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

      uint8_t* ptr = msg.data.data();
      const bool apply_gaussian_noise = gaussian_noise_ != 0;
      if (_msg->scan().ranges_size() != rangeCount*verticalRangeCount)
        ROS_ERROR("3D laser plugin: scan has unexpected size (%d, expected %d)", _msg->scan().ranges_size(), rangeCount*verticalRangeCount);

      for (int hit = 0; hit < rangeCount; hit++)
      {
        // so that it starts from the top angle and goes down (according to Ouster data ordering)
        for (int j = verticalRangeCount-1; j >= 0; j--)
        {
          // ensure that the points are ordered starting from the angle 0 and goes up to 2*pi
          int i = rangeCount/2 - hit;
          if (i < 0)
            i += rangeCount;

          // Range
          double r = _msg->scan().ranges(i + j * rangeCount);

          // Intensity
          const float intensity = _msg->scan().intensities(i + j * rangeCount);

          // Noise
          if (apply_gaussian_noise)
            r += gaussianKernel(0, gaussian_noise_);

          // Ignore points that lay outside range bands or optionally, beneath a
          // minimum intensity level.
          if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY))
          {
            if (ordered_)
              r = 0.0;
            else
              continue;
          }

          const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs_.at(i * verticalRangeCount + j);

          // pAngle is rotated by yAngle:
          *((float*)(ptr + 0)) = r * x_coeff;
          *((float*)(ptr + 4)) = r * y_coeff;
          *((float*)(ptr + 8)) = r * z_coeff;
          *((float*)(ptr + 16)) = intensity;
#if GAZEBO_MAJOR_VERSION > 2
          *((uint8_t*)(ptr + 20)) = j;  // ring
#else
          *((uint8_t*)(ptr + 20)) = verticalRangeCount - 1 - j;  // ring
#endif
          *((uint32_t*)(ptr + 24)) = static_cast<uint32_t>(1000.0*r);  // ring
          ptr += POINT_STEP;
        }
      }

      // Populate message with number of valid points
      msg.point_step = POINT_STEP;
      msg.row_step = ptr - msg.data.data();
      msg.height = 1;
      msg.width = msg.row_step / POINT_STEP;
      msg.is_bigendian = false;
      msg.is_dense = true;
      msg.data.resize(msg.row_step);  // Shrink to actual size

      // Publish output
      {
        boost::lock_guard<boost::mutex> lock(lock_);
        pub_.publish(msg);
        pub_diag_.publish(std_msgs::Empty());
      }
      m_processing_scan = false;
    }

    //}

    // Custom Callback Queue
    ////////////////////////////////////////////////////////////////////////////////
    // Custom callback queue thread
    /* laserQueueThread() //{ */

    void laserQueueThread()
    {
      while (nh_.ok())
      {
        laser_queue_.callAvailable(ros::WallDuration(0.01));
      }
    }

    //}

    /// \brief The parent ray sensor
  private:
    sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief Pointer to ROS node
    ros::NodeHandle nh_;

    /// \brief ROS publisher
    ros::Publisher pub_;

    /// \brief ROS publisher of diagnostics
    ros::Publisher pub_diag_;

    /// \brief topic name
    std::string topic_name_;

    /// \brief topic name of diagnostics
    std::string topic_diag_name_;

    /// \brief Minimum range to publish
    double min_range_;

    /// \brief Maximum range to publish
    double max_range_;

    /// \brief Minimum intensity to publish
    double min_intensity_;

    /// \brief Whether the cloud should be ordered (dense) - invalid points will not be dropped, but will be [0,0,0]
    bool ordered_;

    /// \brief Gaussian noise
    double gaussian_noise_;

    /// \brief Sensor has imu
    bool imu_;

    /// \brief Gaussian noise generator
    static double gaussianKernel(double mu, double sigma)
    {
      // using Box-Muller transform to generate two independent standard normally distributed normal variables
      // see wikipedia
      double U = (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
      double V = (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
      return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
    }

    // | ------------------ TF-related variables ------------------ |
  private:
    std::vector<std::tuple<double, double, double>> coord_coeffs_;

    std::string parent_frame_name_;
    std::string sensor_frame_name_;
    std::string lidar_frame_name_;
    std::string imu_frame_name_;

    /// \brief frame transform parameters
    double sensor_x_, sensor_y_, sensor_z_, sensor_roll_, sensor_pitch_, sensor_yaw_;
    double lidar_x_, lidar_y_, lidar_z_, lidar_roll_, lidar_pitch_, lidar_yaw_;
    double imu_x_, imu_y_, imu_z_, imu_roll_, imu_pitch_, imu_yaw_;

    ros::Publisher tf_pub_;
    tf2_msgs::TFMessage tf_message_;
    ros::WallTimer timer_;
    boost::thread load_thread_;

    // | --------------------- other variables -------------------- |
    /// \brief A mutex to lock access
    boost::mutex lock_;

    /// \brief For setting ROS name space
    std::string robot_namespace_;

    // Custom Callback Queue
    ros::CallbackQueue laser_queue_;
    boost::thread callback_laser_queue_thread_;

    // Subscribe to gazebo laserscan
    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr sub_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRos3DLaser)

}  // namespace gazebo

