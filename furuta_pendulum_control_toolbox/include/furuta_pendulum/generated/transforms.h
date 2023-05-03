#ifndef FURUTAPENDULUM_TRANSFORMS_H_
#define FURUTAPENDULUM_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit
{
namespace FurutaPendulum
{

template <typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl
{

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef typename TRAIT::Scalar Scalar;

  typedef JointState<Scalar> JState;
  class Dummy
  {
  };
  typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;

public:
  class Type_fr_base_link_X_fr_arm1 : public TransformMotion<Scalar, Type_fr_base_link_X_fr_arm1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_arm1();
    const Type_fr_base_link_X_fr_arm1 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_arm2 : public TransformMotion<Scalar, Type_fr_base_link_X_fr_arm2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_arm2();
    const Type_fr_base_link_X_fr_arm2 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_ee : public TransformMotion<Scalar, Type_fr_base_link_X_fr_ee>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_ee();
    const Type_fr_base_link_X_fr_ee & update(const JState &);

  protected:
  };

  class Type_fr_arm2_X_fr_base_link : public TransformMotion<Scalar, Type_fr_arm2_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm2_X_fr_base_link();
    const Type_fr_arm2_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_arm1_X_fr_base_link : public TransformMotion<Scalar, Type_fr_arm1_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm1_X_fr_base_link();
    const Type_fr_arm1_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_ee_X_fr_base_link : public TransformMotion<Scalar, Type_fr_ee_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_ee_X_fr_base_link();
    const Type_fr_ee_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_joint1
  : public TransformMotion<Scalar, Type_fr_base_link_X_fr_joint1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_joint1();
    const Type_fr_base_link_X_fr_joint1 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_joint2
  : public TransformMotion<Scalar, Type_fr_base_link_X_fr_joint2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_joint2();
    const Type_fr_base_link_X_fr_joint2 & update(const JState &);

  protected:
  };

  class Type_fr_arm2_X_fr_arm1 : public TransformMotion<Scalar, Type_fr_arm2_X_fr_arm1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm2_X_fr_arm1();
    const Type_fr_arm2_X_fr_arm1 & update(const JState &);

  protected:
  };

  class Type_fr_arm1_X_fr_arm2 : public TransformMotion<Scalar, Type_fr_arm1_X_fr_arm2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm1_X_fr_arm2();
    const Type_fr_arm1_X_fr_arm2 & update(const JState &);

  protected:
  };

public:
  MotionTransforms();
  void updateParameters();
  Type_fr_base_link_X_fr_arm1 fr_base_link_X_fr_arm1;
  Type_fr_base_link_X_fr_arm2 fr_base_link_X_fr_arm2;
  Type_fr_base_link_X_fr_ee fr_base_link_X_fr_ee;
  Type_fr_arm2_X_fr_base_link fr_arm2_X_fr_base_link;
  Type_fr_arm1_X_fr_base_link fr_arm1_X_fr_base_link;
  Type_fr_ee_X_fr_base_link fr_ee_X_fr_base_link;
  Type_fr_base_link_X_fr_joint1 fr_base_link_X_fr_joint1;
  Type_fr_base_link_X_fr_joint2 fr_base_link_X_fr_joint2;
  Type_fr_arm2_X_fr_arm1 fr_arm2_X_fr_arm1;
  Type_fr_arm1_X_fr_arm2 fr_arm1_X_fr_arm2;

protected:
};  //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef typename TRAIT::Scalar Scalar;

  typedef JointState<Scalar> JState;
  class Dummy
  {
  };
  typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;

public:
  class Type_fr_base_link_X_fr_arm1 : public TransformForce<Scalar, Type_fr_base_link_X_fr_arm1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_arm1();
    const Type_fr_base_link_X_fr_arm1 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_arm2 : public TransformForce<Scalar, Type_fr_base_link_X_fr_arm2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_arm2();
    const Type_fr_base_link_X_fr_arm2 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_ee : public TransformForce<Scalar, Type_fr_base_link_X_fr_ee>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_ee();
    const Type_fr_base_link_X_fr_ee & update(const JState &);

  protected:
  };

  class Type_fr_arm2_X_fr_base_link : public TransformForce<Scalar, Type_fr_arm2_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm2_X_fr_base_link();
    const Type_fr_arm2_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_arm1_X_fr_base_link : public TransformForce<Scalar, Type_fr_arm1_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm1_X_fr_base_link();
    const Type_fr_arm1_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_ee_X_fr_base_link : public TransformForce<Scalar, Type_fr_ee_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_ee_X_fr_base_link();
    const Type_fr_ee_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_joint1 : public TransformForce<Scalar, Type_fr_base_link_X_fr_joint1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_joint1();
    const Type_fr_base_link_X_fr_joint1 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_joint2 : public TransformForce<Scalar, Type_fr_base_link_X_fr_joint2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_joint2();
    const Type_fr_base_link_X_fr_joint2 & update(const JState &);

  protected:
  };

  class Type_fr_arm2_X_fr_arm1 : public TransformForce<Scalar, Type_fr_arm2_X_fr_arm1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm2_X_fr_arm1();
    const Type_fr_arm2_X_fr_arm1 & update(const JState &);

  protected:
  };

  class Type_fr_arm1_X_fr_arm2 : public TransformForce<Scalar, Type_fr_arm1_X_fr_arm2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm1_X_fr_arm2();
    const Type_fr_arm1_X_fr_arm2 & update(const JState &);

  protected:
  };

public:
  ForceTransforms();
  void updateParameters();
  Type_fr_base_link_X_fr_arm1 fr_base_link_X_fr_arm1;
  Type_fr_base_link_X_fr_arm2 fr_base_link_X_fr_arm2;
  Type_fr_base_link_X_fr_ee fr_base_link_X_fr_ee;
  Type_fr_arm2_X_fr_base_link fr_arm2_X_fr_base_link;
  Type_fr_arm1_X_fr_base_link fr_arm1_X_fr_base_link;
  Type_fr_ee_X_fr_base_link fr_ee_X_fr_base_link;
  Type_fr_base_link_X_fr_joint1 fr_base_link_X_fr_joint1;
  Type_fr_base_link_X_fr_joint2 fr_base_link_X_fr_joint2;
  Type_fr_arm2_X_fr_arm1 fr_arm2_X_fr_arm1;
  Type_fr_arm1_X_fr_arm2 fr_arm1_X_fr_arm2;

protected:
};  //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef typename TRAIT::Scalar Scalar;

  typedef JointState<Scalar> JState;
  class Dummy
  {
  };
  typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;

public:
  class Type_fr_base_link_X_fr_arm1
  : public TransformHomogeneous<Scalar, Type_fr_base_link_X_fr_arm1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_arm1();
    const Type_fr_base_link_X_fr_arm1 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_arm2
  : public TransformHomogeneous<Scalar, Type_fr_base_link_X_fr_arm2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_arm2();
    const Type_fr_base_link_X_fr_arm2 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_ee : public TransformHomogeneous<Scalar, Type_fr_base_link_X_fr_ee>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_ee();
    const Type_fr_base_link_X_fr_ee & update(const JState &);

  protected:
  };

  class Type_fr_arm2_X_fr_base_link
  : public TransformHomogeneous<Scalar, Type_fr_arm2_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm2_X_fr_base_link();
    const Type_fr_arm2_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_arm1_X_fr_base_link
  : public TransformHomogeneous<Scalar, Type_fr_arm1_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm1_X_fr_base_link();
    const Type_fr_arm1_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_ee_X_fr_base_link : public TransformHomogeneous<Scalar, Type_fr_ee_X_fr_base_link>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_ee_X_fr_base_link();
    const Type_fr_ee_X_fr_base_link & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_joint1
  : public TransformHomogeneous<Scalar, Type_fr_base_link_X_fr_joint1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_joint1();
    const Type_fr_base_link_X_fr_joint1 & update(const JState &);

  protected:
  };

  class Type_fr_base_link_X_fr_joint2
  : public TransformHomogeneous<Scalar, Type_fr_base_link_X_fr_joint2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_base_link_X_fr_joint2();
    const Type_fr_base_link_X_fr_joint2 & update(const JState &);

  protected:
  };

  class Type_fr_arm2_X_fr_arm1 : public TransformHomogeneous<Scalar, Type_fr_arm2_X_fr_arm1>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm2_X_fr_arm1();
    const Type_fr_arm2_X_fr_arm1 & update(const JState &);

  protected:
  };

  class Type_fr_arm1_X_fr_arm2 : public TransformHomogeneous<Scalar, Type_fr_arm1_X_fr_arm2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Type_fr_arm1_X_fr_arm2();
    const Type_fr_arm1_X_fr_arm2 & update(const JState &);

  protected:
  };

public:
  HomogeneousTransforms();
  void updateParameters();
  Type_fr_base_link_X_fr_arm1 fr_base_link_X_fr_arm1;
  Type_fr_base_link_X_fr_arm2 fr_base_link_X_fr_arm2;
  Type_fr_base_link_X_fr_ee fr_base_link_X_fr_ee;
  Type_fr_arm2_X_fr_base_link fr_arm2_X_fr_base_link;
  Type_fr_arm1_X_fr_base_link fr_arm1_X_fr_base_link;
  Type_fr_ee_X_fr_base_link fr_ee_X_fr_base_link;
  Type_fr_base_link_X_fr_joint1 fr_base_link_X_fr_joint1;
  Type_fr_base_link_X_fr_joint2 fr_base_link_X_fr_joint2;
  Type_fr_arm2_X_fr_arm1 fr_arm2_X_fr_arm1;
  Type_fr_arm1_X_fr_arm2 fr_arm1_X_fr_arm2;

protected:
};  //class 'HomogeneousTransforms'

}  // namespace tpl

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}  // namespace FurutaPendulum
}  // namespace iit

#include "transforms.impl.h"

#endif
