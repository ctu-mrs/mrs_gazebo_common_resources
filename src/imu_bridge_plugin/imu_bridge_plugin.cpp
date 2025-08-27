#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/msgs/imu.pb.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace gazebo
{
  class ImuBridgePlugin : public ModelPlugin
  {
  public:
    ImuBridgePlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      std::cout << "[ImuBridgePlugin] Loading..." << std::endl; 
      model = _model;

      // Initialize ROS node handle (only once)
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "imu_bridge_plugin_node", ros::init_options::NoSigintHandler);
      }
      rosNode.reset(new ros::NodeHandle("imu_bridge_plugin_node"));

      // Get the IMU Gazebo topic name from SDF parameter
      if (_sdf->HasElement("imu_gazebo_topic"))
        imuGazeboTopic = _sdf->Get<std::string>("imu_gazebo_topic");
      else {
        std::cout << "[ImuBridgePlugin] No 'imu_gazebo_topic' specified in SDF. Using default '" << imuGazeboTopic << "'." << std::endl; 
      }
      
      // Get the IMU output topic name from SDF parameter
      std::string outputTopic = "/imu_hector/data";
      if (_sdf->HasElement("output_topic"))
        outputTopic = _sdf->Get<std::string>("output_topic");
      else {
        std::cout << "[ImuBridgePlugin] No 'output_topic' specified in SDF. Using default '" << outputTopic << "'." << std::endl; 
      }

      // Setup ROS publisher
      imuRosPub = rosNode->advertise<sensor_msgs::Imu>(outputTopic, 10);

      // Create Gazebo transport node
      gzNode = transport::NodePtr(new transport::Node());
      gzNode->Init();

      // Subscribe to Gazebo IMU topic
      imuSub = gzNode->Subscribe(imuGazeboTopic, &ImuBridgePlugin::OnImuMsg, this);

      std::cout << "[ImuBridgePlugin] Loaded, subscribing to Gazebo topic: " << imuGazeboTopic << ", publishing to Ros topic: " << outputTopic <<  std::endl; 
    }

  private:
    void OnImuMsg(ConstIMUPtr &msg)
    {
      // Convert gazebo::msgs::IMU to sensor_msgs::Imu
      // std::cout << "[ImuBridgePlugin] Received IMU message from Gazebo topic: " << imuGazeboTopic << std::endl;

      if (!msg)
      {
        ROS_ERROR_THROTTLE(1.0, "[ImuBridgePlugin] Received null message from Gazebo IMU topic: %s", imuGazeboTopic.c_str());
        return;
      }

      sensor_msgs::Imu imuMsg;

      // Orientation (if provided)
      imuMsg.orientation.x = msg->orientation().x();
      imuMsg.orientation.y = msg->orientation().y();
      imuMsg.orientation.z = msg->orientation().z();
      imuMsg.orientation.w = msg->orientation().w();
      
      // Angular velocity
      imuMsg.angular_velocity.x = msg->angular_velocity().x();
      imuMsg.angular_velocity.y = msg->angular_velocity().y();
      imuMsg.angular_velocity.z = msg->angular_velocity().z();

      // Linear acceleration
      imuMsg.linear_acceleration.x = msg->linear_acceleration().x();
      imuMsg.linear_acceleration.y = msg->linear_acceleration().y();
      imuMsg.linear_acceleration.z = msg->linear_acceleration().z();

      imuMsg.header.stamp.sec = msg->stamp().sec();
      imuMsg.header.stamp.nsec = msg->stamp().nsec();
      imuMsg.header.frame_id = "/uav1/vio_imu";
    
      imuRosPub.publish(imuMsg);
    }

    physics::ModelPtr model;
    transport::NodePtr gzNode;
    transport::SubscriberPtr imuSub;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher imuRosPub;

    std::string imuGazeboTopic = "/gazebo/default/imu";
  };

  GZ_REGISTER_MODEL_PLUGIN(ImuBridgePlugin)
}
