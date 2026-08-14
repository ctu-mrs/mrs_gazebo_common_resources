#include "mrs_gazebo_common_resources/ardupilot_propeller_plugin.h"

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <sdf/Element.hh>

// Register plugin at global scope: the macro injects a `namespace gz {...}`
// block, which must land on ::gz (calling it inside the systems namespace
// creates the bogus gz::sim::systems::gz alias that then shadows gz::common
// in the logging macros).
GZ_ADD_PLUGIN(gz::sim::systems::ArduPilotPropeller,
              gz::sim::System,
              gz::sim::systems::ArduPilotPropeller::ISystemConfigure,
              gz::sim::systems::ArduPilotPropeller::ISystemPreUpdate)
// Alias so SDF can name the plugin without the version namespace
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ArduPilotPropeller, "ArduPilotPropeller")

using gz::math::Vector3d;

void gz::sim::systems::ArduPilotPropeller::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  model = Model(_entity);

  const std::string jointName = _sdf->Get<std::string>("jointName");
  const std::string linkName = _sdf->Get<std::string>("linkName");

  if (_sdf->HasElement("force_constant"))
    forceConstant = _sdf->Get<double>("force_constant");
  if (_sdf->HasElement("moment_constant"))
    momentConstant = _sdf->Get<double>("moment_constant");
  if (_sdf->HasElement("turningDirection"))
  {
    turningDirection = _sdf->Get<std::string>("turningDirection") == "cw" ? -1 : 1;
  }

  jointEntity = model.JointByName(_ecm, jointName);
  if (jointEntity == kNullEntity)
  {
    gzerr << "ArduPilotPropeller: could not find joint [" << jointName << "]\n";
    return;
  }

  propLinkEntity = model.LinkByName(_ecm, linkName);
  if (propLinkEntity == kNullEntity)
  {
    gzerr << "ArduPilotPropeller: could not find link [" << linkName << "]\n";
    return;
  }

  // Reaction torque acts on the arm/body link the prop joint hangs from.
  auto parentLinkName = Joint(jointEntity).ParentLinkName(_ecm);
  if (parentLinkName.has_value())
  {
    parentLinkEntity = model.LinkByName(_ecm, *parentLinkName);
  }
  if (parentLinkEntity == kNullEntity)
  {
    gzwarn << "ArduPilotPropeller: parent link not found, reaction torque "
              "falls back to the prop link\n";
    parentLinkEntity = propLinkEntity;
  }

  gzdbg << "ArduPilotPropeller configured for " << jointName
        << " (forceConstant=" << forceConstant
        << ", momentConstant=" << momentConstant
        << ", turningDirection=" << turningDirection << ")\n";
}

void gz::sim::systems::ArduPilotPropeller::PreUpdate(
    const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  if (jointEntity == kNullEntity || propLinkEntity == kNullEntity)
    return;

  double omega = 0.0;
  const auto *vComp = _ecm.Component<components::JointVelocity>(jointEntity);
  if (vComp != nullptr && !vComp->Data().empty())
    omega = vComp->Data()[0];

  if (omega == 0.0)
    return;

  const double force = forceConstant * omega * omega;
  // Aerodynamic reaction torque opposes the actual rotation, so the body
  // receives -sign(omega) * moment * F (CCW prop -> -z, CW prop -> +z).
  const double signedTorque =
      -(omega > 0.0 ? 1.0 : -1.0) * momentConstant * force;

  // world z of the prop link so lift follows the (possibly tilted) frame
  const auto pose = Link(propLinkEntity).WorldPose(_ecm);
  const Vector3d liftDir =
      pose.has_value() ? pose->Rot() * Vector3d::UnitZ : Vector3d::UnitZ;

  Link(propLinkEntity).AddWorldWrench(_ecm, force * liftDir, Vector3d::Zero);
  Link(parentLinkEntity).AddWorldWrench(_ecm, Vector3d::Zero,
                                        signedTorque * liftDir);
}
