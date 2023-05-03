#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::FurutaPendulum;
using namespace iit::FurutaPendulum::dyn;

iit::rbd::Vector3d iit::FurutaPendulum::getWholeBodyCOM(
  const InertiaProperties & inertiaProps, const HomogeneousTransforms & ht)
{
  iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

  HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
  tmpX = tmpX * ht.fr_base_link_X_fr_arm1;
  tmpSum +=
    inertiaProps.getMass_arm1() * (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_arm1()));

  tmpX = tmpX * ht.fr_arm1_X_fr_arm2;
  tmpSum +=
    inertiaProps.getMass_arm2() * (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_arm2()));

  return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::FurutaPendulum::getWholeBodyCOM(
  const InertiaProperties & inertiaProps, const JointState & q, HomogeneousTransforms & ht)
{
  // First updates the coordinate transforms that will be used by the routine
  ht.fr_base_link_X_fr_arm1(q);
  ht.fr_arm1_X_fr_arm2(q);

  // The actual calculus
  return getWholeBodyCOM(inertiaProps, ht);
}
