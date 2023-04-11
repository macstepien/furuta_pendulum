// Initialization of static-const data
template <typename TRAIT>
const typename iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    arm1_I(inertiaProps->getTensor_arm1() ),
    arm2_I(inertiaProps->getTensor_arm2() )
    ,
        base_link_I( inertiaProps->getTensor_base_link() ),
        arm2_Ic(arm2_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot FurutaPendulum, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    arm1_v.setZero();
    arm2_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& base_link_a,
    const Acceleration& g, const Velocity& base_link_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_link_Ic = base_link_I;
    arm1_Ic = arm1_I;

    // First pass, link 'arm1'
    arm1_v = ((xm->fr_arm1_X_fr_base_link) * base_link_v);
    arm1_v(iit::rbd::AZ) += qd(JOINT1);
    
    iit::rbd::motionCrossProductMx<Scalar>(arm1_v, vcross);
    
    arm1_a = (vcross.col(iit::rbd::AZ) * qd(JOINT1));
    arm1_a(iit::rbd::AZ) += qdd(JOINT1);
    
    arm1_f = arm1_I * arm1_a + iit::rbd::vxIv(arm1_v, arm1_I);
    
    // First pass, link 'arm2'
    arm2_v = ((xm->fr_arm2_X_fr_arm1) * arm1_v);
    arm2_v(iit::rbd::AZ) += qd(JOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(arm2_v, vcross);
    
    arm2_a = (xm->fr_arm2_X_fr_arm1) * arm1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    arm2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    arm2_f = arm2_I * arm2_a + iit::rbd::vxIv(arm2_v, arm2_I);
    
    // The force exerted on the floating base by the links
    base_link_f = iit::rbd::vxIv(base_link_v, base_link_I);
    

    // Add the external forces:
    base_link_f -= fext[BASE_LINK];
    arm1_f -= fext[ARM1];
    arm2_f -= fext[ARM2];

    arm1_Ic = arm1_Ic + (xm->fr_arm2_X_fr_arm1).transpose() * arm2_Ic * (xm->fr_arm2_X_fr_arm1);
    arm1_f = arm1_f + (xm->fr_arm2_X_fr_arm1).transpose() * arm2_f;
    
    base_link_Ic = base_link_Ic + (xm->fr_arm1_X_fr_base_link).transpose() * arm1_Ic * (xm->fr_arm1_X_fr_base_link);
    base_link_f = base_link_f + (xm->fr_arm1_X_fr_base_link).transpose() * arm1_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_link_a = - base_link_Ic.inverse() * base_link_f;
    
    arm1_a = xm->fr_arm1_X_fr_base_link * base_link_a;
    jForces(JOINT1) = (arm1_Ic.row(iit::rbd::AZ) * arm1_a + arm1_f(iit::rbd::AZ));
    
    arm2_a = xm->fr_arm2_X_fr_arm1 * arm1_a;
    jForces(JOINT2) = (arm2_Ic.row(iit::rbd::AZ) * arm2_a + arm2_f(iit::rbd::AZ));
    

    base_link_a += g;
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_link_a = -g;

    // Link 'arm1'
    arm1_a = (xm->fr_arm1_X_fr_base_link) * base_link_a;
    arm1_f = arm1_I * arm1_a;
    // Link 'arm2'
    arm2_a = (xm->fr_arm2_X_fr_arm1) * arm1_a;
    arm2_f = arm2_I * arm2_a;

    base_link_f = base_link_I * base_link_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_v, const JointState& qd)
{
    // Link 'arm1'
    arm1_v = ((xm->fr_arm1_X_fr_base_link) * base_link_v);
    arm1_v(iit::rbd::AZ) += qd(JOINT1);
    iit::rbd::motionCrossProductMx<Scalar>(arm1_v, vcross);
    arm1_a = (vcross.col(iit::rbd::AZ) * qd(JOINT1));
    arm1_f = arm1_I * arm1_a + iit::rbd::vxIv(arm1_v, arm1_I);
    
    // Link 'arm2'
    arm2_v = ((xm->fr_arm2_X_fr_arm1) * arm1_v);
    arm2_v(iit::rbd::AZ) += qd(JOINT2);
    iit::rbd::motionCrossProductMx<Scalar>(arm2_v, vcross);
    arm2_a = (xm->fr_arm2_X_fr_arm1) * arm1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    arm2_f = arm2_I * arm2_a + iit::rbd::vxIv(arm2_v, arm2_I);
    

    base_link_f = iit::rbd::vxIv(base_link_v, base_link_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_link_a = baseAccel -g;

    // First pass, link 'arm1'
    arm1_v = ((xm->fr_arm1_X_fr_base_link) * base_link_v);
    arm1_v(iit::rbd::AZ) += qd(JOINT1);
    
    iit::rbd::motionCrossProductMx<Scalar>(arm1_v, vcross);
    
    arm1_a = (xm->fr_arm1_X_fr_base_link) * base_link_a + vcross.col(iit::rbd::AZ) * qd(JOINT1);
    arm1_a(iit::rbd::AZ) += qdd(JOINT1);
    
    arm1_f = arm1_I * arm1_a + iit::rbd::vxIv(arm1_v, arm1_I) - fext[ARM1];
    
    // First pass, link 'arm2'
    arm2_v = ((xm->fr_arm2_X_fr_arm1) * arm1_v);
    arm2_v(iit::rbd::AZ) += qd(JOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(arm2_v, vcross);
    
    arm2_a = (xm->fr_arm2_X_fr_arm1) * arm1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    arm2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    arm2_f = arm2_I * arm2_a + iit::rbd::vxIv(arm2_v, arm2_I) - fext[ARM2];
    

    // The base
    base_link_f = base_link_I * base_link_a + iit::rbd::vxIv(base_link_v, base_link_I) - fext[BASE_LINK];

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'arm2'
    jForces(JOINT2) = arm2_f(iit::rbd::AZ);
    arm1_f += xm->fr_arm2_X_fr_arm1.transpose() * arm2_f;
    // Link 'arm1'
    jForces(JOINT1) = arm1_f(iit::rbd::AZ);
    base_link_f += xm->fr_arm1_X_fr_base_link.transpose() * arm1_f;
}

