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
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'arm1'
    arm1_a = (xm->fr_arm1_X_fr_base_link).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    arm1_f = arm1_I * arm1_a;
    // Link 'arm2'
    arm2_a = (xm->fr_arm2_X_fr_arm1) * arm1_a;
    arm2_f = arm2_I * arm2_a;

    secondPass(jForces);
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'arm1'
    arm1_v(iit::rbd::AZ) = qd(JOINT1);   // arm1_v = vJ, for the first link of a fixed base robot
    
    arm1_f = iit::rbd::vxIv(qd(JOINT1), arm1_I);
    
    // Link 'arm2'
    arm2_v = ((xm->fr_arm2_X_fr_arm1) * arm1_v);
    arm2_v(iit::rbd::AZ) += qd(JOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(arm2_v, vcross);
    
    arm2_a = (vcross.col(iit::rbd::AZ) * qd(JOINT2));
    
    arm2_f = arm2_I * arm2_a + iit::rbd::vxIv(arm2_v, arm2_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'arm1'
    arm1_a = (xm->fr_arm1_X_fr_base_link).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    arm1_a(iit::rbd::AZ) += qdd(JOINT1);
    arm1_v(iit::rbd::AZ) = qd(JOINT1);   // arm1_v = vJ, for the first link of a fixed base robot
    
    arm1_f = arm1_I * arm1_a + iit::rbd::vxIv(qd(JOINT1), arm1_I)  - fext[ARM1];
    
    // First pass, link 'arm2'
    arm2_v = ((xm->fr_arm2_X_fr_arm1) * arm1_v);
    arm2_v(iit::rbd::AZ) += qd(JOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(arm2_v, vcross);
    
    arm2_a = (xm->fr_arm2_X_fr_arm1) * arm1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    arm2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    arm2_f = arm2_I * arm2_a + iit::rbd::vxIv(arm2_v, arm2_I) - fext[ARM2];
    
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{
    // Link 'arm2'
    jForces(JOINT2) = arm2_f(iit::rbd::AZ);
    arm1_f += xm->fr_arm2_X_fr_arm1.transpose() * arm2_f;
    // Link 'arm1'
    jForces(JOINT1) = arm1_f(iit::rbd::AZ);
}
