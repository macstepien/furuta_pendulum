#ifndef IIT_ROBOT_FURUTAPENDULUM_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_FURUTAPENDULUM_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace FurutaPendulum {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot FurutaPendulum.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_arm1() const;
        const IMatrix& getTensor_arm2() const;
        Scalar getMass_arm1() const;
        Scalar getMass_arm2() const;
        const Vec3d& getCOM_arm1() const;
        const Vec3d& getCOM_arm2() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_arm1;
        IMatrix tensor_arm2;
        Vec3d com_arm1;
        Vec3d com_arm2;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_arm1() const {
    return this->tensor_arm1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_arm2() const {
    return this->tensor_arm2;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_arm1() const {
    return this->tensor_arm1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_arm2() const {
    return this->tensor_arm2.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_arm1() const {
    return this->com_arm1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_arm2() const {
    return this->com_arm2;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 0.3 + 0.075;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
