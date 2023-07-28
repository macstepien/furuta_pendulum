// Based on ocs2 ballbot example

#pragma once

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

// furuta_pendulum
#include "furuta_pendulum_ocs2/definitions.h"

namespace ocs2
{
namespace furuta_pendulum
{

class FurutaPendulumSystemDynamics : public SystemDynamicsBaseAD
{
public:
  FurutaPendulumSystemDynamics(const std::string & libraryFolder, bool recompileLibraries)
  : SystemDynamicsBaseAD()
  {
    initialize(
      STATE_DIM, INPUT_DIM, "furuta_pendulum_dynamics", libraryFolder, recompileLibraries, true);
  }

  ~FurutaPendulumSystemDynamics() override = default;

  FurutaPendulumSystemDynamics(const FurutaPendulumSystemDynamics & rhs) = default;

  FurutaPendulumSystemDynamics * clone() const override
  {
    return new FurutaPendulumSystemDynamics(*this);
  }

  ad_vector_t systemFlowMap(
    ad_scalar_t time, const ad_vector_t & state, const ad_vector_t & input,
    const ad_vector_t & parameters) const override;
};

}  // namespace furuta_pendulum
}  // namespace ocs2
