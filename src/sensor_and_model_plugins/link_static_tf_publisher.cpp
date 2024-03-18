/* Mostly generated using ChatGPT 3.5 */

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace gazebo
{

class LinkStaticTFPublisher : public ModelPlugin {
private:
  physics::ModelPtr                   model;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;

public:
  LinkStaticTFPublisher() {
  }

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Safety check
    if (!_model) {
      ROS_ERROR("[LinkStaticTFPublisher]: Invalid model pointer!");
      return;
    }

    model = _model;

    std::string parentLinkName;
    std::string childLinkName;
    std::string robotNamespace;

    robotNamespace = "/";
    if (_sdf->HasElement("robotNamespace")) {
        robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    if (_sdf->HasElement("parentLink")) {
      parentLinkName = _sdf->Get<std::string>("parentLink");
    }else{
      ROS_ERROR("[LinkStaticTFPublisher]: SDF is missing element \"parentLink\"");
      return;
    }

    if (_sdf->HasElement("childLink")) {
      childLinkName = _sdf->Get<std::string>("childLink");
    }else{
      ROS_ERROR("[LinkStaticTFPublisher]: SDF is missing element \"childLink\"");
      return;
    }

    // Get pointers to the specified parent and child links
    physics::LinkPtr parentLink = model->GetLink(parentLinkName);
    physics::LinkPtr childLink  = model->GetLink(childLinkName);

    // Check if both links are valid
    if (!parentLink || !childLink) {
      ROS_ERROR_STREAM("[LinkStaticTFPublisher]: One or both of the links \"" << parentLinkName << "\", \"" << childLinkName << "\" do not exist!");
      return;
    }

    // Get the transform between the parent and child links
    ignition::math::Pose3d relativePose = childLink->WorldPose() - parentLink->WorldPose();

    tf2::Quaternion quat(relativePose.Rot().X(), relativePose.Rot().Y(), relativePose.Rot().Z(), relativePose.Rot().W());

    // Publish the transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp            = ros::Time::now();
    transformStamped.header.frame_id         = robotNamespace + parentLinkName.erase(parentLinkName.find("_link"), std::string("_link").length());
    transformStamped.child_frame_id          = robotNamespace + childLinkName.erase(childLinkName.find("_link"), std::string("_link").length());
    transformStamped.transform.translation.x = relativePose.Pos().X();
    transformStamped.transform.translation.y = relativePose.Pos().Y();
    transformStamped.transform.translation.z = relativePose.Pos().Z();
    transformStamped.transform.rotation.w    = relativePose.Rot().W();
    transformStamped.transform.rotation.x    = relativePose.Rot().X();
    transformStamped.transform.rotation.y    = relativePose.Rot().Y();
    transformStamped.transform.rotation.z    = relativePose.Rot().Z();
    tf_broadcaster.sendTransform(transformStamped);
    ROS_INFO_STREAM("[LinkStaticTFPublisher]: Published static TF between frames \"" << robotNamespace << parentLinkName << "\", \""  << robotNamespace << childLinkName << "\"");
  }
};

GZ_REGISTER_MODEL_PLUGIN(LinkStaticTFPublisher)
}  // namespace gazebo
