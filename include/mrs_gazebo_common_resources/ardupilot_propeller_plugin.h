/*
 * Parametric propeller thrust/reaction model for the ArduPilot SITL backend.
 *
 * Replaces the gz-sim LiftDrag aerodynamics for spinning propellers: LiftDrag
 * needs the blade sweep velocity to be sampled fine enough, and at the 200 Hz
 * physics rate of the MRS worlds a rotor at ~440 rad/s (126 deg per step)
 * delivers only a few percent of the expected force (measured ~3% at 200 Hz,
 * ~48% at 1 kHz). This system instead applies the same parametric thrust the
 * MRS system is tuned around:
 *
 *   F_z = forceConstant * omega^2            (up, sign independent)
 *   tau_z(parent) = -spin_sign * momentConstant * F_z
 *
 * The propeller joint itself is driven by the ArduPilotPlugin VELOCITY force
 * PID as usual - this system only reads the resulting joint velocity.
 *
 * One instance per propeller. All SDF entity names are model-relative and
 * unscoped so that the model can be spawned under any name.
 *
 * The shared library is named `ArduPilotPropeller` (not lib-prefixed in SDF),
 * so gz-sim resolves the plugin by that exact name through the
 * GZ_SIM_SYSTEM_PLUGIN_PATH hook of this package.
 */

#ifndef MRS_GAZEBO_COMMON_RESOURCES_ARDUPILOT_PROPELLER_PLUGIN_H
#define MRS_GAZEBO_COMMON_RESOURCES_ARDUPILOT_PROPELLER_PLUGIN_H

#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
namespace systems
{

class GZ_SIM_VISIBLE ArduPilotPropeller : public System,
                                          public ISystemConfigure,
                                          public ISystemPreUpdate
{
  /// \brief Configure from SDF, resolve joint/link/parent entities.
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) override;

  /// \brief Apply thrust + reaction torque from the current joint velocity.
  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override;

  /// \brief Propeller joint being driven by the controller.
  public: Entity jointEntity{kNullEntity};

  /// \brief Propeller link receiving the lift force.
  public: Entity propLinkEntity{kNullEntity};

  /// \brief Parent (arm/body) link receiving the reaction torque.
  public: Entity parentLinkEntity{kNullEntity};

  /// \brief Model for scoped lookups.
  public: Model model{kNullEntity};

  /// \brief Thrust coefficient [N/(rad/s)^2]. MRS force_constant.
  public: double forceConstant{0.000042};

  /// \brief Reaction torque ratio [N*m / N]. MRS moment_constant.
  public: double momentConstant{0.06};

  /// \brief Joint spin sign: +1 CCW (positive joint velocity), -1 CW.
  /// Kept for documentation/validation; the reaction torque follows the
  /// actual (signed) joint velocity, so it is robust to either mounting.
  public: int turningDirection{1};
};

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif  // MRS_GAZEBO_COMMON_RESOURCES_ARDUPILOT_PROPELLER_PLUGIN_H
