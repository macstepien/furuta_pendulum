// Based on ocs2 ballbot example

#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_slp/SlpSettings.h>
#include <ocs2_sqp/SqpSettings.h>

// Ballbot
#include "furuta_pendulum_ocs2/dynamics/FurutaPendulumSystemDynamics.h"

namespace ocs2
{
namespace furuta_pendulum
{

/**
 * FurutaPendulumInterface class
 * General interface for mpc implementation on the furuta_pendulum model
 */
class FurutaPendulumInterface final : public RobotInterface
{
public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   */
  FurutaPendulumInterface(const std::string & taskFile, const std::string & libraryFolder);

  /**
   * Destructor
   */
  ~FurutaPendulumInterface() override = default;

  const vector_t & getInitialState() { return initialState_; }

  slp::Settings & slpSettings() { return slpSettings_; }

  sqp::Settings & sqpSettings() { return sqpSettings_; }

  ddp::Settings & ddpSettings() { return ddpSettings_; }

  mpc::Settings & mpcSettings() { return mpcSettings_; }

  const OptimalControlProblem & getOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override
  {
    return referenceManagerPtr_;
  }

  const RolloutBase & getRollout() const { return *rolloutPtr_; }

  const Initializer & getInitializer() const override { return *furutaPendulumInitializerPtr_; }

private:
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;
  slp::Settings slpSettings_;

  OptimalControlProblem problem_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> furutaPendulumInitializerPtr_;

  vector_t initialState_{STATE_DIM};
};

}  // namespace furuta_pendulum
}  // namespace ocs2
