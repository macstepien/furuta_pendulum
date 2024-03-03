/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <furuta_pendulum_ocs2/FurutaPendulumInterface.h>
#include <furuta_pendulum_ocs2/definitions.h>

#include "furuta_pendulum_ocs2_ros/FurutaPendulumDummyVisualization.h"

int main(int argc, char **argv)
{
  const std::string robotName = "furuta_pendulum";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nodeHandle = std::make_shared<rclcpp::Node>("furuta_pendulum_mrt");

  // Robot interface
  std::string taskFile =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "config" / "mpc" / "task.info";
  std::string libraryFolder =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "auto_generated";
  ocs2::furuta_pendulum::FurutaPendulumInterface furutaPendulumInterface(taskFile, libraryFolder);

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&furutaPendulumInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto furutaPendulumDummyVisualization = std::make_shared<ocs2::furuta_pendulum::FurutaPendulumDummyVisualization>(nodeHandle);

  // Dummy loop
  ocs2::MRT_ROS_Dummy_Loop dummyFurutaPendulum(mrt, furutaPendulumInterface.mpcSettings().mrtDesiredFrequency_,
                                         furutaPendulumInterface.mpcSettings().mpcDesiredFrequency_);
  dummyFurutaPendulum.subscribeObservers({furutaPendulumDummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = furutaPendulumInterface.getInitialState();
  initObservation.input.setZero(ocs2::furuta_pendulum::INPUT_DIM);
  initObservation.time = 0.0;

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {furutaPendulumInterface.getInitialTarget()},
                                                        {ocs2::vector_t::Zero(ocs2::furuta_pendulum::INPUT_DIM)});

  // Run dummy (loops while ros is ok)
  dummyFurutaPendulum.run(initObservation, initTargetTrajectories);

  return 0;
}
