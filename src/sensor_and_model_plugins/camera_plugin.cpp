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

/*
 @mainpage
   Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

namespace gazebo
{

  class GazeboRosCamera : public CameraPlugin, GazeboRosCameraUtils
  {

  public:
    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    /* GazeboRosCamera() constructor //{ */
    GazeboRosCamera()
    {
    }
    //}

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    /* ~GazeboRosCamera() destructor //{ */
    ~GazeboRosCamera()
    {
      ROS_DEBUG_STREAM_NAMED("camera", "Unloaded");
    }
    //}

    ////////////////////////////////////////////////////////////////////////////////
    /* Load() method //{ */
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
    
      CameraPlugin::Load(_parent, _sdf);
      // copying from CameraPlugin into GazeboRosCameraUtils
      this->parentSensor_ = this->parentSensor;
      this->width_ = this->width;
      this->height_ = this->height;
      this->depth_ = this->depth;
      this->format_ = this->format;
      this->camera_ = this->camera;
    
      GazeboRosCameraUtils::Load(_parent, _sdf);
    
      if (!_sdf->HasElement("parentFrameName"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <parentFrameName>, defaults to \"world\"");
        this->parent_frame_name_ = "world";
      } else
        this->parent_frame_name_ = _sdf->Get<std::string>("parentFrameName");
    
      if (!_sdf->HasElement("sensorBaseFrameName"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <sensorBaseFrameName>, defaults to \"sensor_base\"");
        this->base_frame_name_ = "sensor_base";
      } else
        this->base_frame_name_ = _sdf->Get<std::string>("sensorBaseFrameName");
    
      if (!_sdf->HasElement("x"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <x>, defaults to 0");
        this->x_ = 0;
      } else
        this->x_ = _sdf->Get<double>("x");
    
      if (!_sdf->HasElement("y"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <y>, defaults to 0");
        this->y_ = 0;
      } else
        this->y_ = _sdf->Get<double>("y");
    
      if (!_sdf->HasElement("z"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <z>, defaults to 0");
        this->z_ = 0;
      } else
        this->z_ = _sdf->Get<double>("z");
    
      if (!_sdf->HasElement("roll"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <roll>, defaults to 0");
        this->roll_ = 0;
      } else
        this->roll_ = _sdf->Get<double>("roll");
    
      if (!_sdf->HasElement("pitch"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <pitch>, defaults to 0");
        this->pitch_ = 0;
      } else
        this->pitch_ = _sdf->Get<double>("pitch");
    
      if (!_sdf->HasElement("yaw"))
      {
        ROS_INFO_NAMED("camera", "camera plugin missing <yaw>, defaults to 0");
        this->yaw_ = 0;
      } else
        this->yaw_ = _sdf->Get<double>("yaw");
    
      this->load_thread_ = boost::thread(boost::bind(&GazeboRosCamera::transformThread, this));
    }
    //}

  private:
    ////////////////////////////////////////////////////////////////////////////////
    /* transformThread() method //{ */
    void transformThread()
    {
      while (!this->initialized_)
      {
        ros::Duration(0.01).sleep();
      }
      ROS_INFO_NAMED("camera", "camera plugin - GazeboRosCameraUtils initialized");
      this->tf_pub_ = this->rosnode_->advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 10, false);
    
      createStaticTransforms();
      this->timer_ = this->rosnode_->createWallTimer(ros::WallDuration(1.0), &GazeboRosCamera::publishStaticTransforms, this);
    }
    //}

    ////////////////////////////////////////////////////////////////////////////////
    /* createStaticTransforms() method //{ */
    void createStaticTransforms()
    {
      // frame describing sensor main baseline frame in drone frame
      geometry_msgs::TransformStamped static_transform_base_world;
    
      // frame describing camera's baseline frames in sensor main baseline
      geometry_msgs::TransformStamped static_transform_camera_base;
    
      static_transform_base_world.header.stamp = ros::Time::now();
      static_transform_base_world.header.frame_id = this->parent_frame_name_;
      static_transform_base_world.child_frame_id = tf::resolve(std::string(), this->base_frame_name_);
      static_transform_base_world.transform.translation.x = this->x_;
      static_transform_base_world.transform.translation.y = this->y_;
      static_transform_base_world.transform.translation.z = this->z_;
      tf2::Quaternion quat;
      quat.setRPY(this->roll_, this->pitch_, this->yaw_);
      static_transform_base_world.transform.rotation.x = quat.x();
      static_transform_base_world.transform.rotation.y = quat.y();
      static_transform_base_world.transform.rotation.z = quat.z();
      static_transform_base_world.transform.rotation.w = quat.w();
    
      static_transform_camera_base.header.stamp = static_transform_base_world.header.stamp;
      static_transform_camera_base.header.frame_id = static_transform_base_world.child_frame_id;
      static_transform_camera_base.child_frame_id = tf::resolve(std::string(), this->frame_name_);
    
      quat.setRPY(-M_PI_2, 0, -M_PI_2);
      static_transform_camera_base.transform.rotation.x = quat.x();
      static_transform_camera_base.transform.rotation.y = quat.y();
      static_transform_camera_base.transform.rotation.z = quat.z();
      static_transform_camera_base.transform.rotation.w = quat.w();
    
      this->tf_message_.transforms.push_back(static_transform_base_world);
      this->tf_message_.transforms.push_back(static_transform_camera_base);
      /* ROS_INFO_NAMED("camera", "created"); */
    }
    //}

    ////////////////////////////////////////////////////////////////////////////////
    /* publishStaticTransforms() method //{ */
    void publishStaticTransforms([[maybe_unused]] const ros::WallTimerEvent& event)
    {
      /* ROS_INFO("publishing"); */
      this->tf_pub_.publish(this->tf_message_);
    }
    //}

  protected:
    ////////////////////////////////////////////////////////////////////////////////
    // Update the controller
    /* OnNewFrame() method //{ */
    void OnNewFrame(const unsigned char* _image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string& _format)
    {
      common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();
    
      if (!this->parentSensor->IsActive())
      {
        if ((*this->image_connect_count_) > 0)
          // do this first so there's chance for sensor to run once after activated
          this->parentSensor->SetActive(true);
      } else
      {
        if ((*this->image_connect_count_) > 0)
        {
          if (sensor_update_time < this->last_update_time_)
          {
            ROS_WARN_NAMED("camera", "Negative sensor update time difference detected.");
            this->last_update_time_ = sensor_update_time;
          }
    
          // OnNewFrame is triggered at the gazebo sensor <update_rate>
          // while there is also a plugin <updateRate> that can throttle the
          // rate down further (but then why not reduce the sensor rate?
          // what is the use case?).
          // Setting the <updateRate> to zero will make this plugin
          // update at the gazebo sensor <update_rate>, update_period_ will be
          // zero and the conditional always will be true.
          if (sensor_update_time - this->last_update_time_ >= this->update_period_)
          {
            this->PutCameraData(_image, sensor_update_time);
            this->PublishCameraInfo(sensor_update_time);
            this->last_update_time_ = sensor_update_time;
          }
        }
      }
    }
    //}


  private:
    std::string parent_frame_name_;
    std::string base_frame_name_;

    /// \brief frame transform parameters
  private:
    double x_, y_, z_, roll_, pitch_, yaw_;

  private:
    ros::Publisher tf_pub_;
    tf2_msgs::TFMessage tf_message_;
    ros::WallTimer timer_;
    boost::thread load_thread_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCamera)

}  // namespace gazebo

