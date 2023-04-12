
// Initialization of static-const data
template <typename TRAIT>
const typename iit::FurutaPendulum::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::FurutaPendulum::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::FurutaPendulum::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::FurutaPendulum::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    arm1_v.setZero();
    arm1_c.setZero();
    arm2_v.setZero();
    arm2_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    arm1_AI = inertiaProps->getTensor_arm1();
    arm1_p = - fext[ARM1];
    arm2_AI = inertiaProps->getTensor_arm2();
    arm2_p = - fext[ARM2];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link arm1
    //  - The spatial velocity:
    arm1_v(iit::rbd::AZ) = qd(JOINT1);
    
    //  - The bias force term:
    arm1_p += iit::rbd::vxIv(qd(JOINT1), arm1_AI);
    
    // + Link arm2
    //  - The spatial velocity:
    arm2_v = (motionTransforms-> fr_arm2_X_fr_arm1) * arm1_v;
    arm2_v(iit::rbd::AZ) += qd(JOINT2);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(arm2_v, vcross);
    arm2_c = vcross.col(iit::rbd::AZ) * qd(JOINT2);
    
    //  - The bias force term:
    arm2_p += iit::rbd::vxIv(arm2_v, arm2_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link arm2
    arm2_u = tau(JOINT2) - arm2_p(iit::rbd::AZ);
    arm2_U = arm2_AI.col(iit::rbd::AZ);
    arm2_D = arm2_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(arm2_AI, arm2_U, arm2_D, Ia_r);  // same as: Ia_r = arm2_AI - arm2_U/arm2_D * arm2_U.transpose();
    pa = arm2_p + Ia_r * arm2_c + arm2_U * arm2_u/arm2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_arm2_X_fr_arm1, IaB);
    arm1_AI += IaB;
    arm1_p += (motionTransforms-> fr_arm2_X_fr_arm1).transpose() * pa;
    
    // + Link arm1
    arm1_u = tau(JOINT1) - arm1_p(iit::rbd::AZ);
    arm1_U = arm1_AI.col(iit::rbd::AZ);
    arm1_D = arm1_U(iit::rbd::AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    arm1_a = (motionTransforms-> fr_arm1_X_fr_base_link).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    qdd(JOINT1) = (arm1_u - arm1_U.dot(arm1_a)) / arm1_D;
    arm1_a(iit::rbd::AZ) += qdd(JOINT1);
    
    arm2_a = (motionTransforms-> fr_arm2_X_fr_arm1) * arm1_a + arm2_c;
    qdd(JOINT2) = (arm2_u - arm2_U.dot(arm2_a)) / arm2_D;
    arm2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    
}
