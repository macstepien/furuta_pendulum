
template <typename TRAIT>
iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_base_link_J_fr_arm2(), 
    fr_base_link_J_fr_arm1(), 
    fr_base_link_J_fr_ee()
{
    updateParameters();
}

template <typename TRAIT>
void iit::FurutaPendulum::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_arm2::Type_fr_base_link_J_fr_arm2()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,1) = 0;
    (*this)(4,1) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_arm2& iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_arm2::update(const JState& jState) {
    Scalar s_q_joint1_;
    Scalar c_q_joint1_;
    
    s_q_joint1_ = TRAIT::sin( jState(JOINT1));
    c_q_joint1_ = TRAIT::cos( jState(JOINT1));
    
    (*this)(0,1) =  c_q_joint1_;
    (*this)(1,1) =  s_q_joint1_;
    (*this)(3,0) = (- 0.278 *  s_q_joint1_);
    (*this)(4,0) = ( 0.278 *  c_q_joint1_);
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_arm1::Type_fr_base_link_J_fr_arm1()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_arm1& iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_arm1::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_ee::Type_fr_base_link_J_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_ee& iit::FurutaPendulum::tpl::Jacobians<TRAIT>::Type_fr_base_link_J_fr_ee::update(const JState& jState) {
    Scalar s_q_joint1_;
    Scalar s_q_joint2_;
    Scalar c_q_joint1_;
    Scalar c_q_joint2_;
    
    s_q_joint1_ = TRAIT::sin( jState(JOINT1));
    s_q_joint2_ = TRAIT::sin( jState(JOINT2));
    c_q_joint1_ = TRAIT::cos( jState(JOINT1));
    c_q_joint2_ = TRAIT::cos( jState(JOINT2));
    
    (*this)(0,1) =  c_q_joint1_;
    (*this)(1,1) =  s_q_joint1_;
    (*this)(3,0) = (((- 0.3 *  c_q_joint1_) *  s_q_joint2_) - ( 0.278 *  s_q_joint1_));
    (*this)(3,1) = ((- 0.3 *  s_q_joint1_) *  c_q_joint2_);
    (*this)(4,0) = (( 0.278 *  c_q_joint1_) - (( 0.3 *  s_q_joint1_) *  s_q_joint2_));
    (*this)(4,1) = (( 0.3 *  c_q_joint1_) *  c_q_joint2_);
    (*this)(5,1) = ( 0.3 *  s_q_joint2_);
    return *this;
}
