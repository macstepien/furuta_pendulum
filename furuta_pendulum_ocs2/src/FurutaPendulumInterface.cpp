// Based on ocs2 ballbot example

#include <iostream>
#include <string>

#include "furuta_pendulum_ocs2/FurutaPendulumInterface.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>

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
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  // DDP SQP MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp");
  slpSettings_ = slp::loadSettings(taskFile, "slp");

  /*
   * ReferenceManager & SolverSynchronizedModule
   */
  referenceManagerPtr_.reset(new ReferenceManager);

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

  std::cerr << "Dynamics\n";
  // Dynamics
  bool recompileLibraries;  // load the flag to generate library files from taskFile
  ocs2::loadData::loadCppDataType(
    taskFile, "furuta_pendulum_interface.recompileLibraries", recompileLibraries);
  problem_.dynamicsPtr.reset(new FurutaPendulumSystemDynamics(libraryFolder, recompileLibraries));

  std::cerr << "Rollout\n";
  // Rollout
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  std::cerr << "Initialization\n";
  // Initialization
  furutaPendulumInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}

}  // namespace furuta_pendulum
}  // namespace ocs2
