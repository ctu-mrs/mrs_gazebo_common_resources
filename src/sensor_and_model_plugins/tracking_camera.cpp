#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

#include <gazebo/rendering/Scene.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

namespace gazebo {
class TrackingCamera : public CameraPlugin, GazeboRosCameraUtils {

public:
  TrackingCamera() {}

  ~TrackingCamera() { ROS_DEBUG_STREAM_NAMED("tracking camera", "Unloaded"); }

  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM_NAMED(
          "tracking camera",
          "A ROS node for Gazebo has not been initialized, unable to load "
          "plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
                 "in the gazebo_ros package)");
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

    if (!_sdf->HasElement("x")) {
      ROS_INFO_NAMED("tracking camera",
                     "camera plugin missing <x>, defaults to 0");
      this->x_ = 0;
    } else
      this->x_ = _sdf->Get<double>("x");

    if (!_sdf->HasElement("y")) {
      ROS_INFO_NAMED("tracking camera",
                     "camera plugin missing <y>, defaults to 0");
      this->y_ = 0;
    } else
      this->y_ = _sdf->Get<double>("y");

    if (!_sdf->HasElement("z")) {
      ROS_INFO_NAMED("tracking camera",
                     "camera plugin missing <z>, defaults to 0");
      this->z_ = 0;
    } else
      this->z_ = _sdf->Get<double>("z");

    if (!_sdf->HasElement("roll")) {
      ROS_INFO_NAMED("tracking camera",
                     "camera plugin missing <roll>, defaults to 0");
      this->roll_ = 0;
    } else
      this->roll_ = _sdf->Get<double>("roll");

    if (!_sdf->HasElement("pitch")) {
      ROS_INFO_NAMED("tracking camera",
                     "camera plugin missing <pitch>, defaults to 0");
      this->pitch_ = 0;
    } else
      this->pitch_ = _sdf->Get<double>("pitch");

    if (!_sdf->HasElement("yaw")) {
      ROS_INFO_NAMED("tracking camera",
                     "camera plugin missing <yaw>, defaults to 0");
      this->yaw_ = 0;
    } else
      this->yaw_ = _sdf->Get<double>("yaw");

    this->camera->SetWorldPose(ignition::math::Pose3d(
        ignition::math::Vector3(this->x_, this->y_, this->z_),
        ignition::math::Quaternion(this->roll_, this->pitch_, this->yaw_)));
    ROS_INFO_NAMED("tracking camera", "plugin - TrackingCamera initialized");
  }

protected:
  // Update the controller
  void OnNewFrame(const unsigned char *_image, unsigned int _width,
                  unsigned int _height, unsigned int _depth,
                  const std::string &_format) {
    common::Time sensor_update_time = this->parentSensor->LastMeasurementTime();

    if (!this->parentSensor->IsActive()) {
      if ((*this->image_connect_count_) > 0)
        // do this first so there's chance for sensor to run once after
        // activated
        this->parentSensor->SetActive(true);
    } else {
      if ((*this->image_connect_count_) > 0) {
        if (sensor_update_time < this->last_update_time_) {
          ROS_WARN_NAMED("tracking camera",
                         "Negative sensor update time difference detected.");
          this->last_update_time_ = sensor_update_time;
        }

        // OnNewFrame is triggered at the gazebo sensor <update_rate>
        // while there is also a plugin <updateRate> that can throttle the
        // rate down further (but then why not reduce the sensor rate?
        // what is the use case?).
        // Setting the <updateRate> to zero will make this plugin
        // update at the gazebo sensor <update_rate>, update_period_ will be
        // zero and the conditional always will be true.
        if (sensor_update_time - this->last_update_time_ >=
            this->update_period_) {
          // Assume you have pointers to the camera and parent model
          // ignition::math::Pose3d parent_pose = parent_sensor_->Pose();

          boost::shared_ptr<gazebo::rendering::Scene> scene_ptr =
              this->camera->GetScene();
          std::shared_ptr<gazebo::rendering::Visual> visual_target_ptr =
              scene_ptr->GetVisual("uav1");

          if (visual_target_ptr) {
            ignition::math::Pose3d target_pose = visual_target_ptr->Pose();

            ignition::math::Vector3 rel_position(target_pose.X() - this->x_,
                                                 target_pose.Y() - this->y_,
                                                 target_pose.Z() - this->z_);

            double pitch = std::asin(-rel_position.Z() /
                                     std::sqrt(std::pow(rel_position.X(), 2) +
                                               std::pow(rel_position.Y(), 2) +
                                               std::pow(rel_position.Z(), 2)));
            double yaw = std::atan2(rel_position.Y(), rel_position.X());
            double roll = 0.0;
            ignition::math::Vector3 desired_position(this->x_, this->y_,
                                                     this->z_);
            ignition::math::Quaternion desired_quat(roll, pitch, yaw);
            this->camera->SetWorldPose(
                ignition::math::Pose3d(desired_position, desired_quat));
          } else {
            ROS_WARN_NAMED("tracking camera", "Visual uav1 not found");
          }
          this->PutCameraData(_image, sensor_update_time);
          this->PublishCameraInfo(sensor_update_time);
          this->last_update_time_ = sensor_update_time;
        }
      }
    }
  }

private:
  double x_, y_, z_, roll_, pitch_, yaw_;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(TrackingCamera)

} // namespace gazebo
