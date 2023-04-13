/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/systems/FixBaseSystemBase.h>
#include <ct/rbd/robot/actuator/ActuatorDynamics.h>
#include <ct/rbd/state/FixBaseRobotState.h>

namespace ct
{
namespace rbd
{

/**
 * \brief A fix base rigid body system that uses forward dynamics.
 *
 * A fix base rigid body system that uses forward dynamics. The control input vector is assumed to consist of
 *  - joint torques and
 *  - end effector forces expressed in the world frame
 *
 * The overall state vector is arranged in the order
 * - joint positions
 * - joint velocities
 * - actuator state
 *
 * \warning when modelled with RobCoGen, the base pose must be rotated "against gravity" (RobCoGen modeling assumption)
 */
template <class RBDDynamics>
class FurutaPendulumSystem : public FixBaseSystemBase<
                               RBDDynamics,
                               RBDDynamics::NSTATE,  // state dim
                               1>                    // control dim
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = RBDDynamics::NSTATE;  // combined state dim
  static const size_t CONTROL_DIM = 1;                  // combined control dim
  static const size_t ACTUATOR_STATE_DIM = 0;

  using BASE = FixBaseSystemBase<RBDDynamics, STATE_DIM, 1>;
  using SCALAR = typename BASE::SCALAR;
  using state_vector_t = typename BASE::state_vector_t;
  using control_vector_t = typename BASE::control_vector_t;
  using JointAcceleration_t = typename BASE::JointAcceleration_t;
  using RigidBodyPose_t = typename BASE::RigidBodyPose_t;

  using ActuatorDynamics_t = ActuatorDynamics<ACTUATOR_STATE_DIM, RBDDynamics::NJOINTS, SCALAR>;
  using FixBaseRobotState_t = FixBaseRobotState<BASE::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>;
  using actuator_state_vector_t = typename FixBaseRobotState_t::actuator_state_vector_t;

  /*!
     * @brief constructor
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
  FurutaPendulumSystem(const RigidBodyPose_t & basePose = RigidBodyPose_t())
  : BASE(basePose), actuatorDynamics_(nullptr)
  {
  }
  /*!
     * @brief constructor including actuator dynamics
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
  FurutaPendulumSystem(
    std::shared_ptr<ActuatorDynamics_t> actuatorDynamics,
    const RigidBodyPose_t & basePose = RigidBodyPose_t())
  : BASE(basePose), actuatorDynamics_(actuatorDynamics)
  {
  }

  /*!
     * @brief copy constructor
	 *
	 * @param arg instance of FurutaPendulumSystem to be copied.
	 *
	 * \note takes care of explicitly cloning actuatorDynamics, if existent
	 */
  FurutaPendulumSystem(const FurutaPendulumSystem & arg) : BASE(arg)
  {
    if (arg.actuatorDynamics_) {
      actuatorDynamics_ = std::shared_ptr<ActuatorDynamics_t>(arg.actuatorDynamics_->clone());
    }
  }

  //! destructor
  virtual ~FurutaPendulumSystem() = default;

  //! compute the controlled dynamics of the fixed base robotic system
  void computeControlledDynamics(
    const state_vector_t & state, const SCALAR & t, const control_vector_t & controlIn,
    state_vector_t & derivative) override
  {
    FixBaseRobotState_t robotState(state);

    // map the joint velocities (unchanged, no damping)
    derivative.template topRows<BASE::NJOINTS>() = robotState.joints().getVelocities();

    // temporary variable for the control (will get modified by the actuator dynamics, if applicable)
    control_vector_t control = controlIn;

    // compute actuator dynamics and their control output
    computeActuatorDynamics(robotState, t, controlIn, derivative, control);

    // Cache updated rbd state
    typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

    typename RBDDynamics::JointAcceleration_t jAcc;

    this->dynamics_.FixBaseForwardDynamics(
      robotState.joints(), control.template head<BASE::NJOINTS>(), linkForces, jAcc);

    derivative.template segment<BASE::NJOINTS>(BASE::NJOINTS) = jAcc.getAcceleration();
  }

  //! do nothing if actuators disabled
  void computeActuatorDynamics(
    const FixBaseRobotState_t &, const SCALAR &, const control_vector_t &, state_vector_t &,
    control_vector_t &)
  {
  }

  //! deep cloning
  virtual FurutaPendulumSystem<RBDDynamics> * clone() const override
  {
    return new FurutaPendulumSystem<RBDDynamics>(*this);
  }

  //! get pointer to actuator dynamics
  std::shared_ptr<ActuatorDynamics_t> getActuatorDynamics() { return actuatorDynamics_; }

private:
  //! pointer to the actuator dynamics
  std::shared_ptr<ActuatorDynamics_t> actuatorDynamics_;
};

}  // namespace rbd
}  // namespace ct
