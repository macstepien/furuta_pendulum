// Based on ocs2 ballbot example

#include <furuta_pendulum_ocs2/dynamics/FurutaPendulumSystemDynamics.h>

// robcogen
#include <iit/rbd/rbd.h>
#include <iit/rbd/traits/TraitSelector.h>

#include "furuta_pendulum_ocs2/generated/forward_dynamics.h"
#include "furuta_pendulum_ocs2/generated/inertia_properties.h"
#include "furuta_pendulum_ocs2/generated/inverse_dynamics.h"
#include "furuta_pendulum_ocs2/generated/jsim.h"
#include "furuta_pendulum_ocs2/generated/transforms.h"

namespace ocs2
{
namespace furuta_pendulum
{

ad_vector_t FurutaPendulumSystemDynamics::systemFlowMap(
  ad_scalar_t time, const ad_vector_t & state, const ad_vector_t & input,
  const ad_vector_t & parameters) const
{
  // based on https://www.hindawi.com/journals/jcse/2011/528341/

  // theta3 = dtheta1
  // theta4 = dtheta2
  // dtheta3 = ddtheta1
  // dtheta4 = ddtheta2

  double MAX_MOTOR_VEL = 22.0;

  ad_scalar_t tau1 = input(0);
  ad_scalar_t tau2(0.0);

  // double theta1 = y(0);
  ad_scalar_t theta2 = state(1);
  ad_scalar_t theta3 = state(2);
  ad_scalar_t theta4 = state(3);

  Eigen::Matrix<ad_scalar_t, 5, 1> vec11;
  vec11(0) = -params_.J2_hat_ * params_.b1_;
  vec11(1) = params_.m2_ * params_.L1_ * params_.l2_ * cos(theta2) * params_.b2_;
  vec11(2) = -pow(params_.J2_hat_, 2) * sin(2.0 * theta2);
  vec11(3) = -0.5 * params_.J2_hat_ * params_.m2_ * params_.L1_ * params_.l2_ * cos(theta2) *
             sin(2.0 * theta2);
  vec11(4) = params_.J2_hat_ * params_.m2_ * params_.L1_ * params_.l2_ * sin(theta2);

  Eigen::Matrix<ad_scalar_t, 5, 1> vec21;
  vec21(0) = params_.m2_ * params_.L1_ * params_.l2_ * cos(theta2) * params_.b1_;
  vec21(1) = -params_.b2_ * (params_.J0_hat_ + params_.J2_hat_ * pow(sin(theta2), 2));
  vec21(2) =
    params_.m2_ * params_.L1_ * params_.l2_ * params_.J2_hat_ * cos(theta2) * sin(2.0 * theta2);
  vec21(3) = -0.5 * sin(2.0 * theta2) *
             (params_.J0_hat_ * params_.J2_hat_ + pow(params_.J2_hat_ * sin(theta2), 2));
  vec21(4) = -0.5 * pow(params_.m2_ * params_.L1_ * params_.l2_, 2) * sin(2.0 * theta2);

  Eigen::Matrix<ad_scalar_t, 5, 1> thetas_vec;
  thetas_vec(0) = theta3;
  thetas_vec(1) = theta4;
  thetas_vec(2) = theta3 * theta4;
  thetas_vec(3) = pow(theta3, 2);
  thetas_vec(4) = pow(theta4, 2);

  Eigen::Matrix<ad_scalar_t, 3, 1> vec12;
  vec12(0) = params_.J2_hat_;
  vec12(1) = -params_.m2_ * params_.L1_ * params_.l2_ * cos(theta2);
  vec12(2) = 0.5 * pow(params_.m2_ * params_.l2_, 2) * params_.L1_ * sin(2.0 * theta2);

  Eigen::Matrix<ad_scalar_t, 3, 1> vec22;
  vec22(0) = -params_.m2_ * params_.L1_ * params_.l2_ * cos(theta2);
  vec22(1) = params_.J0_hat_ + params_.J2_hat_ * pow(sin(theta2), 2);
  vec22(2) = -params_.m2_ * params_.l2_ * sin(theta2) *
             (params_.J0_hat_ + params_.J2_hat_ * pow(sin(theta2), 2));

  Eigen::Matrix<ad_scalar_t, 3, 1> taus_g_vec;
  taus_g_vec(0) = tau1;
  taus_g_vec(1) = tau2;
  taus_g_vec(2) = params_.g_;

  ad_scalar_t denominator = params_.J0_hat_ * params_.J2_hat_ +
                            pow(params_.J2_hat_ * sin(theta2), 2) -
                            pow(params_.m2_ * params_.L1_ * params_.l2_ * cos(theta2), 2);

  auto numerator1 = (vec11.transpose() * thetas_vec + vec12.transpose() * taus_g_vec);
  auto numerator2 = (vec21.transpose() * thetas_vec + vec22.transpose() * taus_g_vec);

  ad_vector_t dy(4);
  dy(0) = theta3;
  dy(1) = theta4;
  dy(2) = numerator1(0) / denominator;
  dy(3) = numerator2(0) / denominator;

  // Hard velocity limit, TODO: would be better to model it other way
  if (CppAD::Value(dy(0)).getValue() > MAX_MOTOR_VEL && CppAD::Value(dy(2)).getValue() > 0.0) {
    dy(0) = MAX_MOTOR_VEL;
    dy(2) = ad_scalar_t(0.0);
  } else if (
    CppAD::Value(dy(0)).getValue() < -MAX_MOTOR_VEL && CppAD::Value(dy(2)).getValue() < 0.0) {
    dy(0) = -MAX_MOTOR_VEL;
    dy(2) = ad_scalar_t(0.0);
  }

  return dy;
}

}  // namespace furuta_pendulum
}  // namespace ocs2
