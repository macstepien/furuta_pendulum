#ifndef IIT_ROBOGEN__FURUTAPENDULUM_TRAITS_H_
#define IIT_ROBOGEN__FURUTAPENDULUM_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>

namespace iit
{
namespace FurutaPendulum
{

namespace tpl
{

template <typename SCALAR>
struct Traits
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef SCALAR S;

  typedef typename FurutaPendulum::JointIdentifiers JointID;
  typedef typename FurutaPendulum::LinkIdentifiers LinkID;
  typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

  typedef typename FurutaPendulum::tpl::JointState<SCALAR> JointState;

  typedef typename FurutaPendulum::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
  typedef typename FurutaPendulum::tpl::MotionTransforms<Trait> MotionTransforms;
  typedef typename FurutaPendulum::tpl::ForceTransforms<Trait> ForceTransforms;
  typedef typename FurutaPendulum::tpl::Jacobians<Trait> Jacobians;

  typedef typename iit::FurutaPendulum::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
  typedef typename iit::FurutaPendulum::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
  typedef typename iit::FurutaPendulum::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
  typedef typename iit::FurutaPendulum::dyn::tpl::JSIM<Trait> JSIM;

  static const int joints_count = FurutaPendulum::jointsCount;
  static const int links_count = FurutaPendulum::linksCount;
  static const bool floating_base = false;

  static inline const JointID * orderedJointIDs();
  static inline const LinkID * orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID * Traits<SCALAR>::orderedJointIDs()
{
  return FurutaPendulum::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID * Traits<SCALAR>::orderedLinkIDs()
{
  return FurutaPendulum::orderedLinkIDs;
}

}  // namespace tpl

typedef tpl::Traits<double> Traits;  // default instantiation - backward compatibility...

}  // namespace FurutaPendulum
}  // namespace iit

#endif
