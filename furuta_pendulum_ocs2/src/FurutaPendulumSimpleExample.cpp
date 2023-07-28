// Based on ocs2 ballbot example

// C++
#include <cstdlib>
#include <iostream>
#include <string>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// FurutaPendulum
#include "furuta_pendulum_ocs2/definitions.h"
#include "furuta_pendulum_ocs2/dynamics/FurutaPendulumSystemDynamics.h"

using namespace ocs2;
using namespace furuta_pendulum;

int main(int argc, char ** argv)
{
  /*
   * Setting paths
   */
  // path to config file
  std::string taskFile =
    std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
    "config" / "mpc" / "task.info";

  std::cerr << "Loading task file: " << taskFile << std::endl;
  // path to save auto-generated libraries
  std::string libraryFolder =
    std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
    "auto_generated";

  std::cerr << "Generated library path: " << libraryFolder << std::endl;

  /* The optimal control problem formulation*/
  OptimalControlProblem problem;

  /*
   * DDP settings
   */
  auto ddpSettings = ddp::loadSettings(taskFile, "ddp");
  ddpSettings.displayInfo_ = true;  // display iteration information

  /*
   * Rollout settings
   */
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");

  /*
   * Rollout
   */
  problem.dynamicsPtr.reset(new FurutaPendulumSystemDynamics(libraryFolder, true));
  auto furutaPendulumRolloutPtr =
    std::make_unique<TimeTriggeredRollout>(*problem.dynamicsPtr, rolloutSettings);

  /*
   * Cost function
   */
  matrix_t Q(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  loadData::loadEigenMatrix(taskFile, "R", R);
  matrix_t QFinal(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q_final", QFinal);
  vector_t xInit(STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "initialState", xInit);

  problem.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
  problem.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(QFinal));

  /*
   * Initialization
   */
  auto furutaPendulumInitializerPtr = std::make_unique<DefaultInitializer>(INPUT_DIM);

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  scalar_t timeHorizon;
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", timeHorizon);
  size_t numPartitions;
  loadData::loadCppDataType(taskFile, "mpc.numPartitions", numPartitions);
  scalar_array_t partitioningTimes(numPartitions + 1);
  partitioningTimes[0] = 0.0;
  for (size_t i = 0; i < numPartitions; i++) {
    partitioningTimes[i + 1] = partitioningTimes[i] + timeHorizon / numPartitions;
  }
  partitioningTimes[numPartitions] = timeHorizon;

  /*
   * define solver and run
   */
  ddpSettings.nThreads_ = 1;
  SLQ slq(ddpSettings, *furutaPendulumRolloutPtr, problem, *furutaPendulumInitializerPtr);
  slq.getReferenceManager().setTargetTrajectories(
    TargetTrajectories({0.0}, {xInit}, {vector_t::Zero(INPUT_DIM)}));
  slq.run(0.0, xInit, timeHorizon, partitioningTimes);

  /*
   * Perforce index
   */
  auto performanceIndex = slq.getPerformanceIndeces();

  return 0;
}
