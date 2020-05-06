/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <string>
#include <memory>
#include <random>
#include <chrono>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/distortion_models.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf2_msgs/TFMessage.h>

#include "perlin_noise.h"
#include "common.h"

#define DEPTH_CAMERA_TOPIC "aligned_depth_to_color"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infra1"
#define IRED2_CAMERA_TOPIC "infra2"
#define DEPTH_PUB_FREQ_HZ 30
#define COLOR_PUB_FREQ_HZ 30
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

#define DEPTH_SCALE_M 0.001

namespace gazebo
{

  const std::string DEPTH_CAMERA_SUFFIX = "aligned_depth_to_color";
  const std::string COLOR_CAMERA_SUFFIX = "color";
  const std::string IREDS_CAMERA_SUFFIX = "infra_stereo";
  const std::string IRED1_CAMERA_SUFFIX = "infra1";
  const std::string IRED2_CAMERA_SUFFIX = "infra2";
  const std::string BASE_FRAME_SUFFIX = "link";

  /// \brief A plugin that simulates Real Sense camera streams.
  /*  //{ */

  class RealSensePlugin : public ModelPlugin
  {

    /* dm_upscale() method //{ */
    void dm_upscale(const cv::Mat& in, cv::Mat& out, int scale)
    {
      const unsigned i_rows = in.rows;
      const unsigned i_cols = in.cols;

      const unsigned o_rows = i_rows * scale;
      const unsigned o_cols = i_cols * scale;
      cv::Mat ret = cv::Mat(o_rows, o_cols, CV_16UC1);
      const uint16_t* i_data_ptr = in.ptr<uint16_t>(0);

      /* uint16_t* o_data_ptr = ret.ptr<uint16_t>(0); */
      /* cv::parallel_for_(cv::Range(0, o_rows*o_cols), [&](const cv::Range& range) */
      /* { */
      /*     for (int r = range.start; r < range.end; r++) */
      /*     { */
      /*         const int rit = r / o_cols; */
      /*         const int cit = r - rit*o_cols; */
      /*         o_data_ptr[r] = i_data_ptr[cit/scale + rit/scale*i_cols]; */
      /*     } */
      /* }); */

      uint16_t* o_px_ptr = ret.ptr<uint16_t>(0);
      for (unsigned rit = 0; rit < o_rows; rit++)
      {
        for (unsigned cit = 0; cit < o_cols; cit++)
        {
          *o_px_ptr = i_data_ptr[cit / scale + rit / scale * i_cols];
          o_px_ptr++;
        }
      }
      out = ret;
    }
    //}

  public:
    /////////////////////////////////////////////////
    /* RealSensePlugin() constructor //{ */
    RealSensePlugin()
    {
      this->depthCam = nullptr;
      this->iredStereoCam = nullptr;
      this->colorCam = nullptr;
    }
    //}

    /////////////////////////////////////////////////
    /* Load() method //{ */
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Output the name of the model
      /* std::cout << std::endl */
      /*           << "RealSensePlugin: The realsense_camera plugin is attached to model " */
      /*           << _model->GetName() << std::endl; */
      /* std::cout << "JEDEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEM" << std::endl; */

      // Store a pointer to the this model
      this->rsModel = _model;
      std::string cameraNamespace = _model->GetName();
      std::cout << "Loading RealSensePlugin with namespace " << cameraNamespace << std::endl;

      // Store a pointer to the world
      this->world = this->rsModel->GetWorld();

      // Sensors Manager
      sensors::SensorManager* smanager = sensors::SensorManager::Instance();

      depth_camera_plugin_name_ = cameraNamespace + "/" + DEPTH_CAMERA_SUFFIX;
      color_camera_plugin_name_ = cameraNamespace + "/" + COLOR_CAMERA_SUFFIX;
      ireds_camera_plugin_name_ = cameraNamespace + "/" + IREDS_CAMERA_SUFFIX;
      ired1_camera_plugin_name_ = cameraNamespace + "/" + IRED1_CAMERA_SUFFIX;
      ired2_camera_plugin_name_ = cameraNamespace + "/" + IRED2_CAMERA_SUFFIX;

      // Get Cameras Renderers
      const auto depthPtr = smanager->GetSensor(depth_camera_plugin_name_);
      const auto colorPtr = smanager->GetSensor(color_camera_plugin_name_);
      const auto iredsPtr = smanager->GetSensor(ireds_camera_plugin_name_);

      // Check if camera renderers have been found successfuly
      if (!depthPtr)
      {
        gzerr << "RealSensePlugin: Depth Camera with name \"" << depth_camera_plugin_name_ << "\" has not been found" << std::endl;
        return;
      }
      if (!colorPtr)
      {
        gzerr << "RealSensePlugin: Color Camera with name \"" << color_camera_plugin_name_ << "\" has not been found" << std::endl;
        return;
      }
      if (!iredsPtr)
      {
        gzerr << "RealSensePlugin: InfraRed Stereo Camera with name \"" << ireds_camera_plugin_name_ << "\" has not been found" << std::endl;
        return;
      }

      this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(depthPtr)->DepthCamera();
      this->iredStereoCam = std::dynamic_pointer_cast<sensors::MultiCameraSensor>(iredsPtr);
      this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(colorPtr)->Camera();

      /* std::cout << "number of attributes: " << _sdf->GetAttributeCount() << ", " << _model->GetSDF()->GetAttributeCount() << std::endl; */
      /* std::cout << "useRealistic loaded: " << getSdfParam(_sdf, "useRealistic", this->useRealistic, false) << std::endl; */
      getSdfParam(_sdf, "useRealistic", this->useRealistic, false);

      if (this->useRealistic)
      {
        std::cout << "Using realistic depth sensor" << std::endl;
        getSdfParam(_sdf, "imageScaling", this->scaling, 4u);
        if (this->scaling % 2 && this->scaling != 1)
        {
          gzerr << "Invalid scaling " << this->scaling << ". Scaling has to be divisible by two (or be one)." << std::endl;
          return;
        }

        getSdfParam(_sdf, "noisePerMeter", this->noisePerMeter, 0.2f);
        getSdfParam(_sdf, "noiseMinDistance", this->noiseMinDistance, 4.0f);
        getSdfParam(_sdf, "perlinEmptySpeed", this->perlinEmptySpeed, 0.1f);
        getSdfParam(_sdf, "perlinEmptyThreshold", this->perlinEmptyThreshold, 0.8f);
        getSdfParam(_sdf, "blurSize", this->blurSize, 15u);
        if (this->blurSize % 2 == 0 && this->blurSize != 0)
        {
          gzwarn << "Invalid blur size " << this->blurSize << ". Gaussian blur has to be indivisible by two (or zero to turn off). Changing to default (15)"
                 << std::endl;
          this->blurSize = 15u;
        }
        getSdfParam(_sdf, "erosionSize", this->erosionSize, 5u);


        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        rand_gen = std::default_random_engine(seed);
        /* rand_dist = std::uniform_int_distribution<unsigned int>(0, std::numeric_limits<unsigned int>::max()); */
        randn_dist = std::normal_distribution<float>(0.0f, noisePerMeter / 3);
        perlinNoise = PerlinNoise(seed);
      } else
      {
        std::cout << "Using ideal depth sensor" << std::endl;
        this->scaling = 1;
      }


      // Resize Depth Map dimensions
      try
      {
        if (this->useRealistic)
          this->depthMapSmall.resize(this->depthCam->ImageWidth() * this->depthCam->ImageHeight());
        this->depthMap.resize(this->scaling * this->depthCam->ImageWidth() * this->scaling * this->depthCam->ImageHeight());
      }
      catch (std::bad_alloc& e)
      {
        gzerr << "RealSensePlugin: depthMap allocation failed: " << e.what() << std::endl;
        return;
      }

      // Setup Transport Node
      this->transportNode = transport::NodePtr(new transport::Node());
      this->transportNode->Init(this->world->Name());

      // Setup Publishers
      std::string rsTopicRoot =
          /* "~/rs_d435/"; */
          "~/" + this->rsModel->GetName() + "/rs/stream/";

      this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
      this->ired1Pub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + IRED1_CAMERA_TOPIC, 1, IRED1_PUB_FREQ_HZ);
      this->ired2Pub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + IRED2_CAMERA_TOPIC, 1, IRED2_PUB_FREQ_HZ);
      this->colorPub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + COLOR_CAMERA_TOPIC, 1, COLOR_PUB_FREQ_HZ);

      // Listen to depth camera new frame event
      if (this->useRealistic)
      {
        this->newDepthFrameConn =
            this->depthCam->ConnectNewDepthFrame(std::bind(&RealSensePlugin::OnNewDepthFrameRealistic, this, this->depthCam, this->depthPub));
      } else
      {
        this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(std::bind(&RealSensePlugin::OnNewDepthFrame, this, this->depthCam, this->depthPub));
      }

      // Setup infrared stereo camera
      for (unsigned int i = 0; i < this->iredStereoCam->CameraCount(); ++i)
      {
        this->iredCams.push_back(this->iredStereoCam->Camera(i));

        std::string cameraName = this->iredStereoCam->Camera(i)->Name();
        if (cameraName.find(ired1_camera_plugin_name_) != std::string::npos)
        {
          std::cout << "ired1 found" << std::endl;
          this->newIred1FrameConn = this->iredCams[i]->ConnectNewImageFrame(std::bind(&RealSensePlugin::OnNewFrame, this, this->iredCams[i], this->ired1Pub));
        } else if (cameraName.find(ired2_camera_plugin_name_) != std::string::npos)
        {
          std::cout << "ired2 found" << std::endl;
          this->newIred2FrameConn = this->iredCams[i]->ConnectNewImageFrame(std::bind(&RealSensePlugin::OnNewFrame, this, this->iredCams[i], this->ired2Pub));
        }
      }
      this->iredStereoCam->SetActive(true);

      this->newColorFrameConn = this->colorCam->ConnectNewImageFrame(std::bind(&RealSensePlugin::OnNewFrame, this, this->colorCam, this->colorPub));

      // Listen to the update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RealSensePlugin::OnUpdate, this));
    }
    //}

    /////////////////////////////////////////////////
    /* OnNewFrame() method //{ */
    void OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub)
    {
      msgs::ImageStamped msg;

      // Set Simulation Time
      msgs::Set(msg.mutable_time(), this->world->SimTime());

      // Set Image Dimensions
      msg.mutable_image()->set_width(cam->ImageWidth());
      msg.mutable_image()->set_height(cam->ImageHeight());

      // Set Image Pixel Format
      msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(cam->ImageFormat()));

      // Set Image Data
      msg.mutable_image()->set_step(cam->ImageWidth() * cam->ImageDepth());
      msg.mutable_image()->set_data(cam->ImageData(), cam->ImageDepth() * cam->ImageWidth() * cam->ImageHeight());

      // Publish realsense infrared stream
      pub->Publish(msg);
    }
    //}

    /////////////////////////////////////////////////
    /* OnNewDepthFrame() method //{ */
    void OnNewDepthFrame([[maybe_unused]] const rendering::CameraPtr cam, const transport::PublisherPtr pub)
    {
      // Get Depth Map dimensions
      const unsigned imageWidth = this->depthCam->ImageWidth();
      const unsigned imageHeight = this->depthCam->ImageHeight();
      const unsigned imageSize = imageWidth * imageHeight;

      // Instantiate message
      msgs::ImageStamped msg;

      // Convert Float depth data to RealSense depth data
      const float* depthDataFloat = this->depthCam->DepthData();
      for (unsigned int i = 0; i < imageSize; ++i)
      {
        const float cur_depth = depthDataFloat[i];
        // Check clipping and overflow
        if (cur_depth <= this->depthCam->NearClip() || cur_depth >= this->depthCam->FarClip() || cur_depth > DEPTH_SCALE_M * UINT16_MAX || cur_depth < 0)
        {
          this->depthMap[i] = 0;
        } else
        {
          this->depthMap[i] = (cur_depth) / DEPTH_SCALE_M;
        }
      }

      // Pack realsense scaled depth map
      msgs::Set(msg.mutable_time(), this->world->SimTime());
      msg.mutable_image()->set_width(imageWidth);
      msg.mutable_image()->set_height(imageHeight);
      msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
      msg.mutable_image()->set_step(imageWidth * imageHeight);
      msg.mutable_image()->set_data(this->depthMap.data(), sizeof(uint16_t) * imageSize);

      // Publish realsense scaled depth map
      pub->Publish(msg);
    }
    //}

    /////////////////////////////////////////////////
    /* OnNewDepthFrameRealistic() method //{ */
    void OnNewDepthFrameRealistic([[maybe_unused]] const rendering::CameraPtr cam, const transport::PublisherPtr pub)
    {
      // Get Depth Map dimensions
      const unsigned imageWidth = this->depthCam->ImageWidth();
      const unsigned imageHeight = this->depthCam->ImageHeight();
      const unsigned imageSize = imageWidth * imageHeight;

      /* this->perlinNoise = PerlinNoise(rand_dist(rand_gen)); */
      static float cur_z = 6.0;
      cur_z += this->perlinEmptySpeed;

      // Instantiate message
      msgs::ImageStamped msg;

      // Convert Float depth data to RealSense depth data
      const float* depthDataFloat = this->depthCam->DepthData();
      unsigned x = 0, y = 0;
      for (unsigned int i = 0; i < imageSize; ++i)
      {
        const float x_rel = 5.0f * x / float(imageWidth);
        const float y_rel = 5.0f * y / float(imageHeight);
        const float cur_depth = depthDataFloat[i];
        // Check clipping and overflow
        if (cur_depth <= this->depthCam->NearClip() || cur_depth >= this->depthCam->FarClip() || cur_depth > DEPTH_SCALE_M * UINT16_MAX || cur_depth < 0
            || this->perlinNoise.noise(x_rel, y_rel, cur_z) > this->perlinEmptyThreshold)
        {
          this->depthMapSmall[i] = 0;
        } else
        {
          const float noise_scale = cur_depth > this->noiseMinDistance ? cur_depth : this->noiseMinDistance;
          const float noise = noise_scale * this->randn_dist(rand_gen);
          float noisy_depth = (noise + cur_depth);
          if (noisy_depth < 0 || noisy_depth > DEPTH_SCALE_M * UINT16_MAX)
            noisy_depth = 0;
          this->depthMapSmall[i] = noisy_depth / DEPTH_SCALE_M;
        }
        x++;
        if (x > imageWidth)
        {
          x = 0;
          y++;
        }
      }

      const unsigned n_width = depthCam->ImageWidth() * this->scaling;
      const unsigned n_height = depthCam->ImageHeight() * this->scaling;
      const unsigned n_imsize = n_width * n_height;
      cv::Mat im(depthCam->ImageHeight(), depthCam->ImageWidth(), CV_16UC1, this->depthMapSmall.data());
      dm_upscale(im, im, this->scaling);
      if (this->blurSize != 0)
      {
        cv::Mat mask, blr_im, blr_mask;
        cv::compare(im, 0, mask, cv::CMP_NE);
        cv::GaussianBlur(im, blr_im, cv::Size(this->blurSize, this->blurSize), 2 * this->blurSize);
        cv::GaussianBlur(mask, blr_mask, cv::Size(this->blurSize, this->blurSize), 2 * this->blurSize);
        cv::divide(blr_im, blr_mask, blr_im, 255.0, CV_16UC1);
        blr_im.copyTo(im, mask);
      }
      if (this->erosionSize != 0)
      {
        cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(this->erosionSize, this->erosionSize));
        cv::erode(im, im, element);
      }

      /* dm_blur(im, im, 2); */
      /* cv::resize(im, im, cv::Size(n_width, n_height), cv::INTER_NEAREST); */
      memcpy(this->depthMap.data(), im.ptr(0), sizeof(uint16_t) * n_imsize);

      // Pack realsense scaled depth map
      msgs::Set(msg.mutable_time(), this->world->SimTime());
      msg.mutable_image()->set_width(n_width);
      msg.mutable_image()->set_height(n_height);
      msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
      msg.mutable_image()->set_step(n_width * n_height);
      msg.mutable_image()->set_data(this->depthMap.data(), sizeof(uint16_t) * n_imsize);

      // Publish realsense scaled depth map
      pub->Publish(msg);
    }
    //}

    /////////////////////////////////////////////////
    /* OnUpdate() method //{ */
    void OnUpdate()
    {
    }
    //}

  protected:
    std::string depth_camera_plugin_name_;
    std::string color_camera_plugin_name_;
    std::string ireds_camera_plugin_name_;
    std::string ired1_camera_plugin_name_;
    std::string ired2_camera_plugin_name_;

  protected:
    bool useRealistic;
    unsigned scaling = 4;
    float noisePerMeter;
    float noiseMinDistance;
    float perlinEmptySpeed;
    float perlinEmptyThreshold;
    unsigned blurSize;
    unsigned erosionSize;
    PerlinNoise perlinNoise;
    std::default_random_engine rand_gen;
    /* std::uniform_int_distribution<unsigned int> rand_dist; */
    std::normal_distribution<float> randn_dist;

    /// \brief Pointer to the model containing the plugin.
    physics::ModelPtr rsModel;

    /// \brief Pointer to the world.
    physics::WorldPtr world;

    /// \brief Pointer to the Depth Camera Renderer.
    rendering::DepthCameraPtr depthCam;

    /// \brief Pointer to the Color Camera Renderer.
    rendering::CameraPtr colorCam;

    /// \brief Pointer to the Infrared Stereo Camera Renderer.
    sensors::MultiCameraSensorPtr iredStereoCam;

    /// \brief Vector of pointers to to individual infrared Camera Renderers.
    std::vector<rendering::CameraPtr> iredCams;

    /// \brief Pointer to the transport Node.
    transport::NodePtr transportNode;

    // \brief Store Real Sense depth map data.
    std::vector<uint16_t> depthMap;

    // \brief Store raw Real Sense depth map data.
    std::vector<uint16_t> depthMapSmall;

    /// \brief Pointer to the Depth Publisher.
    transport::PublisherPtr depthPub;

    /// \brief Pointer to the Color Publisher.
    transport::PublisherPtr colorPub;

    /// \brief Pointer to the Infrared Publisher.
    transport::PublisherPtr ired1Pub;

    /// \brief Pointer to the Infrared2 Publisher.
    transport::PublisherPtr ired2Pub;

    /// \brief Pointer to the Depth Camera callback connection.
    event::ConnectionPtr newDepthFrameConn;

    /// \brief Pointer to the Depth Camera callback connection.
    event::ConnectionPtr newIred1FrameConn;

    /// \brief Pointer to the Infrared Camera callback connection.
    event::ConnectionPtr newIred2FrameConn;

    /// \brief Pointer to the Color Camera callback connection.
    event::ConnectionPtr newColorFrameConn;

    /// \brief Pointer to the World Update event connection.
    event::ConnectionPtr updateConnection;
  };

  //}


  /// \brief A plugin that simulates Real Sense camera streams.
  /*  //{ */

  class GazeboRosRealsense : public RealSensePlugin
  {
  public:
    /////////////////////////////////////////////////
    /* GazeboRosRealsense() constructor //{ */
    GazeboRosRealsense()
    {
    }
    //}

    /////////////////////////////////////////////////
    /* ~GazeboRosRealsense() destructor //{ */
    ~GazeboRosRealsense()
    {
      ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
    }
    //}

    /////////////////////////////////////////////////
    /* Load() method //{ */
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      ROS_INFO("Realsense Gazebo ROS plugin loading...");
      RealSensePlugin::Load(_model, _sdf);

      cameraNamespace = _model->GetName();
      ROS_INFO_STREAM("Realsense camera attached to " << cameraNamespace);

      depth_camera_frame_id_ = cameraNamespace + "/" + realsense_prefix + "/" + DEPTH_CAMERA_SUFFIX;
      color_camera_frame_id_ = cameraNamespace + "/" + realsense_prefix + "/" + COLOR_CAMERA_SUFFIX;
      ired1_camera_frame_id_ = cameraNamespace + "/" + realsense_prefix + "/" + IRED1_CAMERA_SUFFIX;
      ired2_camera_frame_id_ = cameraNamespace + "/" + realsense_prefix + "/" + IRED2_CAMERA_SUFFIX;
      base_frame_id_ = cameraNamespace + "/" + realsense_prefix + "/" + BASE_FRAME_SUFFIX;

      depth_camera_optical_frame_id_ = depth_camera_frame_id_ + "_optical";
      color_camera_optical_frame_id_ = color_camera_frame_id_ + "_optical";
      ired1_camera_optical_frame_id_ = ired1_camera_frame_id_ + "_optical";
      ired2_camera_optical_frame_id_ = ired2_camera_frame_id_ + "_optical";

      // Get parameters from sdf
      if (!_sdf->HasElement("parentFrameName"))
      {
        ROS_INFO_NAMED("realsense", "realsense plugin missing <parentFrameName>, defaults to \"world\"");
        this->parent_frame_name_ = "world";
      } else
        this->parent_frame_name_ = _sdf->Get<std::string>("parentFrameName");

      if (!_sdf->HasElement("x"))
      {
        ROS_INFO_NAMED("realsense", "realsense plugin missing <x>, defaults to 0");
        this->x_ = 0;
      } else
        this->x_ = _sdf->Get<double>("x");

      if (!_sdf->HasElement("y"))
      {
        ROS_INFO_NAMED("realsense", "realsense plugin missing <y>, defaults to 0");
        this->y_ = 0;
      } else
        this->y_ = _sdf->Get<double>("y");

      if (!_sdf->HasElement("z"))
      {
        ROS_INFO_NAMED("realsense", "realsense plugin missing <z>, defaults to 0");
        this->z_ = 0;
      } else
        this->z_ = _sdf->Get<double>("z");

      if (!_sdf->HasElement("roll"))
      {
        ROS_INFO_NAMED("realsense", "realsense plugin missing <roll>, defaults to 0");
        this->roll_ = 0;
      } else
        this->roll_ = _sdf->Get<double>("roll");

      if (!_sdf->HasElement("pitch"))
      {
        ROS_INFO_NAMED("realsense", "realsense plugin missing <pitch>, defaults to 0");
        this->pitch_ = 0;
      } else
        this->pitch_ = _sdf->Get<double>("pitch");

      if (!_sdf->HasElement("yaw"))
      {
        ROS_INFO_NAMED("realsense", "realsense plugin missing <yaw>, defaults to 0");
        this->yaw_ = 0;
      } else
        this->yaw_ = _sdf->Get<double>("yaw");

      this->rosnode_ = ros::NodeHandle(cameraNamespace + "/rs_d435");

      // initialize camera_info_manager
      this->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(this->rosnode_, cameraNamespace));

      this->itnode_ = std::make_unique<image_transport::ImageTransport>(this->rosnode_);

      this->color_pub_ = this->itnode_->advertiseCamera("color/image_raw", 2);
      this->ir1_pub_ = this->itnode_->advertiseCamera("infra1/image_raw", 2);
      this->ir2_pub_ = this->itnode_->advertiseCamera("infra2/image_raw", 2);
      this->depth_pub_ = this->itnode_->advertiseCamera("aligned_depth_to_color/image_raw", 2);

      this->tf_pub_ = this->rosnode_.advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 100, true);

      createStaticTransforms();
      this->timer_ = this->rosnode_.createWallTimer(ros::WallDuration(1.0), &GazeboRosRealsense::publishStaticTransforms, this);
    }
    //}

    /////////////////////////////////////////////////
    /* createStaticTransforms() method //{ */
    void createStaticTransforms()
    {

      // frame describing sensor main baseline frame in drone frame
      geometry_msgs::TransformStamped static_transform_base_world;

      // frame describing camera's baseline frames in sensor main baseline
      geometry_msgs::TransformStamped static_transform_depth_base;
      geometry_msgs::TransformStamped static_transform_color_base;
      geometry_msgs::TransformStamped static_transform_ir1_base;
      geometry_msgs::TransformStamped static_transform_ir2_base;

      // optical frames of cameras in their baseline frames
      geometry_msgs::TransformStamped static_transform_depthcam_depth;
      geometry_msgs::TransformStamped static_transform_colorcam_color;
      geometry_msgs::TransformStamped static_transform_ir1cam_ir1;
      geometry_msgs::TransformStamped static_transform_ir2cam_ir2;

      static_transform_base_world.header.stamp = ros::Time::now();
      static_transform_base_world.header.frame_id = this->parent_frame_name_;
      static_transform_base_world.child_frame_id = base_frame_id_;

      static_transform_base_world.transform.translation.x = this->x_;
      static_transform_base_world.transform.translation.y = this->y_;
      static_transform_base_world.transform.translation.z = this->z_;
      tf2::Quaternion quat;
      quat.setRPY(this->roll_, this->pitch_, this->yaw_);
      static_transform_base_world.transform.rotation.x = quat.x();
      static_transform_base_world.transform.rotation.y = quat.y();
      static_transform_base_world.transform.rotation.z = quat.z();
      static_transform_base_world.transform.rotation.w = quat.w();

      static_transform_depth_base.header.stamp = static_transform_base_world.header.stamp;
      static_transform_depth_base.header.frame_id = base_frame_id_;
      static_transform_depth_base.child_frame_id = depth_camera_frame_id_;

      static_transform_depth_base.transform.translation.x = 0;
      static_transform_depth_base.transform.translation.y = -0.0115;
      static_transform_depth_base.transform.translation.z = 0;
      quat.setRPY(0, 0, 0);
      static_transform_depth_base.transform.rotation.x = quat.x();
      static_transform_depth_base.transform.rotation.y = quat.y();
      static_transform_depth_base.transform.rotation.z = quat.z();
      static_transform_depth_base.transform.rotation.w = quat.w();

      static_transform_color_base.header.stamp = static_transform_base_world.header.stamp;
      static_transform_color_base.header.frame_id = base_frame_id_;
      static_transform_color_base.child_frame_id = color_camera_frame_id_;

      static_transform_color_base.transform.translation.x = 0;
      static_transform_color_base.transform.translation.y = -0.0115;
      static_transform_color_base.transform.translation.z = 0;
      static_transform_color_base.transform.rotation = static_transform_depth_base.transform.rotation;

      static_transform_ir1_base.header.stamp = static_transform_base_world.header.stamp;
      static_transform_ir1_base.header.frame_id = base_frame_id_;
      static_transform_ir1_base.child_frame_id = ired1_camera_frame_id_;

      static_transform_ir1_base.transform.translation.x = 0;
      static_transform_ir1_base.transform.translation.y = 0.0175;
      static_transform_ir1_base.transform.translation.z = 0;
      static_transform_ir1_base.transform.rotation = static_transform_depth_base.transform.rotation;

      static_transform_ir2_base.header.stamp = static_transform_base_world.header.stamp;
      static_transform_ir2_base.header.frame_id = base_frame_id_;
      static_transform_ir2_base.child_frame_id = ired2_camera_frame_id_;

      static_transform_ir2_base.transform.translation.x = 0;
      static_transform_ir2_base.transform.translation.y = -0.0325;
      static_transform_ir2_base.transform.translation.z = 0;
      static_transform_ir2_base.transform.rotation = static_transform_depth_base.transform.rotation;

      static_transform_depthcam_depth.header.stamp = static_transform_base_world.header.stamp;
      static_transform_depthcam_depth.header.frame_id = depth_camera_frame_id_;
      static_transform_depthcam_depth.child_frame_id = depth_camera_optical_frame_id_;

      quat.setRPY(-M_PI_2, 0, -M_PI_2);
      static_transform_depthcam_depth.transform.rotation.x = quat.x();
      static_transform_depthcam_depth.transform.rotation.y = quat.y();
      static_transform_depthcam_depth.transform.rotation.z = quat.z();
      static_transform_depthcam_depth.transform.rotation.w = quat.w();

      static_transform_colorcam_color.header.stamp = static_transform_base_world.header.stamp;
      static_transform_colorcam_color.header.frame_id = color_camera_frame_id_;
      static_transform_colorcam_color.child_frame_id = color_camera_optical_frame_id_;

      static_transform_colorcam_color.transform.rotation = static_transform_depthcam_depth.transform.rotation;

      static_transform_ir1cam_ir1.header.stamp = static_transform_base_world.header.stamp;
      static_transform_ir1cam_ir1.header.frame_id = ired1_camera_frame_id_;
      static_transform_ir1cam_ir1.child_frame_id = ired1_camera_optical_frame_id_;

      static_transform_ir1cam_ir1.transform.rotation = static_transform_depthcam_depth.transform.rotation;

      static_transform_ir2cam_ir2.header.stamp = static_transform_base_world.header.stamp;
      static_transform_ir2cam_ir2.header.frame_id = ired2_camera_frame_id_;
      static_transform_ir2cam_ir2.child_frame_id = ired2_camera_optical_frame_id_;

      static_transform_ir2cam_ir2.transform.rotation = static_transform_depthcam_depth.transform.rotation;

      this->tf_message_.transforms.push_back(static_transform_base_world);
      this->tf_message_.transforms.push_back(static_transform_depth_base);
      this->tf_message_.transforms.push_back(static_transform_color_base);
      this->tf_message_.transforms.push_back(static_transform_ir1_base);
      this->tf_message_.transforms.push_back(static_transform_ir2_base);
      this->tf_message_.transforms.push_back(static_transform_depthcam_depth);
      this->tf_message_.transforms.push_back(static_transform_colorcam_color);
      this->tf_message_.transforms.push_back(static_transform_ir1cam_ir1);
      this->tf_message_.transforms.push_back(static_transform_ir2cam_ir2);
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

    /////////////////////////////////////////////////
    /* OnNewFrame() method //{ */
    void OnNewFrame(const rendering::CameraPtr cam, [[maybe_unused]] const transport::PublisherPtr pub)
    {
      common::Time current_time = this->world->SimTime();

      // identify camera
      std::string camera_id = cam->Name();
      image_transport::CameraPublisher* image_pub;
      if (camera_id.find(color_camera_plugin_name_) != std::string::npos)
      {
        camera_id = color_camera_optical_frame_id_;
        image_pub = &(this->color_pub_);
      } else if (camera_id.find(ired1_camera_plugin_name_) != std::string::npos)
      {
        camera_id = ired1_camera_optical_frame_id_;
        image_pub = &(this->ir1_pub_);
      } else if (camera_id.find(ired2_camera_plugin_name_) != std::string::npos)
      {
        camera_id = ired2_camera_optical_frame_id_;
        image_pub = &(this->ir2_pub_);
      } else if (camera_id.find(depth_camera_plugin_name_) != std::string::npos)
      {
        camera_id = depth_camera_optical_frame_id_;
        image_pub = &(this->depth_pub_);
      } else
      {
        ROS_ERROR("[GazeboRosrealsense]: Unknown camera name: %s\n", camera_id.c_str());
        camera_id = color_camera_optical_frame_id_;
        image_pub = &(this->color_pub_);
      }

      // copy data into image
      this->image_msg_.header.frame_id = camera_id;
      this->image_msg_.header.stamp.sec = current_time.sec;
      this->image_msg_.header.stamp.nsec = current_time.nsec;

      // set image encoding
      std::string pixel_format = cam->ImageFormat();
      if (pixel_format == "L_INT8")
      {
        pixel_format = sensor_msgs::image_encodings::MONO8;
      } else if (pixel_format == "RGB_INT8")
      {
        pixel_format = sensor_msgs::image_encodings::RGB8;
      } else
      {
        ROS_ERROR("Unsupported Gazebo ImageFormat\n");
        pixel_format = sensor_msgs::image_encodings::BGR8;
      }

      // copy from simulation image to ROS msg
      fillImage(this->image_msg_, pixel_format, cam->ImageHeight(), cam->ImageWidth(), cam->ImageDepth() * cam->ImageWidth(),
                reinterpret_cast<const void*>(cam->ImageData()));

      sensor_msgs::CameraInfo cam_info_msg = cameraInfo(this->image_msg_, cam);

      // publish to ROS
      image_pub->publish(this->image_msg_, cam_info_msg);
    }
    //}

    /////////////////////////////////////////////////
    /* OnNewDepthFrame() method //{ */
    void OnNewDepthFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub)
    {
      // get current time
      common::Time current_time = this->world->SimTime();

      RealSensePlugin::OnNewDepthFrame(cam, pub);

      // copy data into image
      this->depth_msg_.header.frame_id = depth_camera_optical_frame_id_;
      this->depth_msg_.header.stamp.sec = current_time.sec;
      this->depth_msg_.header.stamp.nsec = current_time.nsec;

      // set image encoding
      std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

      // copy from simulation image to ROS msg
      fillImage(this->depth_msg_, pixel_format, this->scaling * this->depthCam->ImageHeight(), this->scaling * this->depthCam->ImageWidth(),
                sizeof(uint16_t) * this->scaling * this->depthCam->ImageWidth(), reinterpret_cast<const void*>(this->depthMap.data()));

      sensor_msgs::CameraInfo depth_info_msg = cameraInfo(this->depth_msg_, cam);

      // publish to ROS
      this->depth_pub_.publish(this->depth_msg_, depth_info_msg);
    }
    //}

    /////////////////////////////////////////////////
    /* OnNewDepthFrameRealistic() method //{ */
    void OnNewDepthFrameRealistic(const rendering::CameraPtr cam, const transport::PublisherPtr pub)
    {
      // get current time
      common::Time current_time = this->world->SimTime();

      RealSensePlugin::OnNewDepthFrameRealistic(cam, pub);

      // copy data into image
      this->depth_msg_.header.frame_id = depth_camera_optical_frame_id_;
      this->depth_msg_.header.stamp.sec = current_time.sec;
      this->depth_msg_.header.stamp.nsec = current_time.nsec;

      // set image encoding
      std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

      // copy from simulation image to ROS msg
      fillImage(this->depth_msg_, pixel_format, this->scaling * this->depthCam->ImageHeight(), this->scaling * this->depthCam->ImageWidth(),
                sizeof(uint16_t) * this->scaling * this->depthCam->ImageWidth(), reinterpret_cast<const void*>(this->depthMap.data()));

      sensor_msgs::CameraInfo depth_info_msg = cameraInfo(this->depth_msg_, cam);

      // publish to ROS
      this->depth_pub_.publish(this->depth_msg_, depth_info_msg);
    }
    //}

    /* cameraInfo() method //{ */
    sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image, const gazebo::rendering::CameraPtr cam)
    {
      sensor_msgs::CameraInfo info_msg;

      info_msg.header = image.header;
      info_msg.height = image.height;
      info_msg.width = image.width;

      double hfov = cam->HFOV().Radian();
      double hfoc = (image.width / 2.0) / tan(hfov / 2.0);
      double vfov = cam->VFOV().Radian();
      double vfoc = (image.height / 2.0) / tan(vfov / 2.0);
      /* float focal = 463.889; */

      info_msg.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
      info_msg.K[0] = hfoc;
      info_msg.K[4] = vfoc;
      info_msg.K[2] = (info_msg.width + 1.0) / 2.0;
      info_msg.K[5] = (info_msg.height + 1.0) / 2.0;
      info_msg.K[8] = 1.;

      info_msg.P[0] = info_msg.K[0];
      info_msg.P[5] = info_msg.K[4];
      info_msg.P[2] = info_msg.K[2];
      info_msg.P[6] = info_msg.K[5];
      info_msg.P[10] = info_msg.K[8];

      return info_msg;
    }
    //}

  private:
    std::string cameraNamespace = "not_linked";
    std::string realsense_prefix = "rs_d435";

  protected:
    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    /// \brief A pointer to the ROS node.
    ///  A node will be instantiated if it does not exist.
  protected:
    ros::NodeHandle rosnode_;

  private:
    std::unique_ptr<image_transport::ImageTransport> itnode_;

  protected:
    image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

  private:
    ros::Publisher tf_pub_;
    tf2_msgs::TFMessage tf_message_;
    ros::WallTimer timer_;

    /// \brief ROS image messages
  protected:
    sensor_msgs::Image image_msg_, depth_msg_;
    int namespace_, frame_id_;

    /// \brief frame transform parameters
    std::string parent_frame_name_;
    double x_, y_, z_, roll_, pitch_, yaw_;

    std::string depth_camera_frame_id_;
    std::string color_camera_frame_id_;
    std::string ired1_camera_frame_id_;
    std::string ired2_camera_frame_id_;
    std::string base_frame_id_;

    std::string depth_camera_optical_frame_id_;
    std::string color_camera_optical_frame_id_;
    std::string ired1_camera_optical_frame_id_;
    std::string ired2_camera_optical_frame_id_;
  };

  //}

  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)
}  // namespace gazebo
