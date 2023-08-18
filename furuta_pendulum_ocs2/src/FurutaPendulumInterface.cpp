// Based on ocs2 ballbot example

#include <iostream>
#include <string>

#include "furuta_pendulum_ocs2/FurutaPendulumInterface.h"
#include "furuta_pendulum_ocs2/dynamics/FurutaPendulumSystemDynamics.h"
#include "furuta_pendulum_ocs2/FurutaPendulumParameters.h"

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2
{
namespace furuta_pendulum
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FurutaPendulumInterface::FurutaPendulumInterface(
  const std::string & taskFile, const std::string & libraryFolder)
{
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[FurutaPendulumInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument(
      "[FurutaPendulumInterface] Task file not found: " + taskFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[FurutaPendulumInterface] Generated library path: " << libraryFolderPath
            << std::endl;

  // Default initial condition
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);
  std::cerr << "x_init:   " << initialState_.transpose() << "\n";
  std::cerr << "x_final:  " << xFinal_.transpose() << "\n";

  // DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", true);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", true);

  /*
   * Optimal control problem
   */
  // Cost
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  loadData::loadEigenMatrix(taskFile, "R", R);
  loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
  std::cerr << "Q:  \n" << Q << "\n";
  std::cerr << "R:  \n" << R << "\n";
  std::cerr << "Q_final:\n" << Qf << "\n";

  problem_.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
  problem_.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Qf));

  // Dynamics
  std::cerr << "Dynamics\n";
  bool recompileLibraries;  // load the flag to generate library files from taskFile
  ocs2::loadData::loadCppDataType(
    taskFile, "furuta_pendulum_interface.recompileLibraries", recompileLibraries);
  FurutaPendulumParameters furutaPendulumParameters;
  furutaPendulumParameters.loadSettings(taskFile, "furuta_pendulum_parameters", true);
  problem_.dynamicsPtr.reset(
    new FurutaPendulumSystemDynamics(furutaPendulumParameters, libraryFolder, recompileLibraries));

  // Rollout
  std::cerr << "Rollout\n";
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", true);
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Constraints
  auto getPenalty = [&]() {
    // one can use either augmented::SlacknessSquaredHingePenalty or augmented::ModifiedRelaxedBarrierPenalty
    using penalty_type = augmented::SlacknessSquaredHingePenalty;
    // using penalty_type = augmented::ModifiedRelaxedBarrierPenalty;
    penalty_type::Config boundsConfig;
    loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, true);
    return penalty_type::create(boundsConfig);
  };

  double controlSignalBound = 0.0;
  ocs2::loadData::loadCppDataType(taskFile, "control_signal_bound", controlSignalBound);

  double joint0VelocityBound = 0.0;
  ocs2::loadData::loadCppDataType(taskFile, "joint0_velocity_bound", joint0VelocityBound);

  auto getConstraint = [&]() {
    // C * x + D * u + e = 0
    constexpr size_t numIneqConstraint = 4;
    const vector_t e = (vector_t(numIneqConstraint) << controlSignalBound, controlSignalBound,
                        joint0VelocityBound, joint0VelocityBound)
                         .finished();
    const vector_t D = (vector_t(numIneqConstraint) << 1.0, -1.0, 0.0, 0.0).finished();
    matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
    C(2, 2) = 1.0;
    C(3, 2) = -1.0;
    return std::make_unique<LinearStateInputConstraint>(e, C, D);
  };
  problem_.inequalityLagrangianPtr->add("InputAndStateLimits", create(getConstraint(), getPenalty()));

  // Initialization
  std::cerr << "Initialization\n";
  furutaPendulumInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}

}  // namespace furuta_pendulum
}  // namespace ocs2
