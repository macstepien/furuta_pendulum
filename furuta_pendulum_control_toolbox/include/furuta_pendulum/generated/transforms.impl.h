
// Constructors
template <typename TRAIT>
iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_arm1_X_fr_base_link(),
    fr_base_link_X_fr_arm1(),
    fr_arm2_X_fr_arm1(),
    fr_arm1_X_fr_arm2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_arm1_X_fr_base_link(),
    fr_base_link_X_fr_arm1(),
    fr_arm2_X_fr_arm1(),
    fr_arm1_X_fr_arm2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_arm1_X_fr_base_link(),
    fr_base_link_X_fr_arm1(),
    fr_arm2_X_fr_arm1(),
    fr_arm1_X_fr_arm2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link::Type_fr_arm1_X_fr_base_link()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link& iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link::update(const JState& q) {
    Scalar s_q_joint1_;
    Scalar c_q_joint1_;
    
    s_q_joint1_ = TRAIT::sin( q(JOINT1));
    c_q_joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,0) =  c_q_joint1_;
    (*this)(0,1) =  s_q_joint1_;
    (*this)(1,0) = - s_q_joint1_;
    (*this)(1,1) =  c_q_joint1_;
    (*this)(3,0) = (- 0.37 *  s_q_joint1_);
    (*this)(3,1) = ( 0.37 *  c_q_joint1_);
    (*this)(3,3) =  c_q_joint1_;
    (*this)(3,4) =  s_q_joint1_;
    (*this)(4,0) = (- 0.37 *  c_q_joint1_);
    (*this)(4,1) = (- 0.37 *  s_q_joint1_);
    (*this)(4,3) = - s_q_joint1_;
    (*this)(4,4) =  c_q_joint1_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1::Type_fr_base_link_X_fr_arm1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1& iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1::update(const JState& q) {
    Scalar s_q_joint1_;
    Scalar c_q_joint1_;
    
    s_q_joint1_ = TRAIT::sin( q(JOINT1));
    c_q_joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,0) =  c_q_joint1_;
    (*this)(0,1) = - s_q_joint1_;
    (*this)(1,0) =  s_q_joint1_;
    (*this)(1,1) =  c_q_joint1_;
    (*this)(3,0) = (- 0.37 *  s_q_joint1_);
    (*this)(3,1) = (- 0.37 *  c_q_joint1_);
    (*this)(3,3) =  c_q_joint1_;
    (*this)(3,4) = - s_q_joint1_;
    (*this)(4,0) = ( 0.37 *  c_q_joint1_);
    (*this)(4,1) = (- 0.37 *  s_q_joint1_);
    (*this)(4,3) =  s_q_joint1_;
    (*this)(4,4) =  c_q_joint1_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1::Type_fr_arm2_X_fr_arm1()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1& iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1::update(const JState& q) {
    Scalar s_q_joint2_;
    Scalar c_q_joint2_;
    
    s_q_joint2_ = TRAIT::sin( q(JOINT2));
    c_q_joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) =  s_q_joint2_;
    (*this)(0,2) = - c_q_joint2_;
    (*this)(1,1) =  c_q_joint2_;
    (*this)(1,2) =  s_q_joint2_;
    (*this)(3,1) = ( 0.278 *  c_q_joint2_);
    (*this)(3,2) = ( 0.278 *  s_q_joint2_);
    (*this)(3,4) =  s_q_joint2_;
    (*this)(3,5) = - c_q_joint2_;
    (*this)(4,1) = (- 0.278 *  s_q_joint2_);
    (*this)(4,2) = ( 0.278 *  c_q_joint2_);
    (*this)(4,4) =  c_q_joint2_;
    (*this)(4,5) =  s_q_joint2_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2::Type_fr_arm1_X_fr_arm2()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2& iit::FurutaPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2::update(const JState& q) {
    Scalar s_q_joint2_;
    Scalar c_q_joint2_;
    
    s_q_joint2_ = TRAIT::sin( q(JOINT2));
    c_q_joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(1,0) =  s_q_joint2_;
    (*this)(1,1) =  c_q_joint2_;
    (*this)(2,0) = - c_q_joint2_;
    (*this)(2,1) =  s_q_joint2_;
    (*this)(4,0) = ( 0.278 *  c_q_joint2_);
    (*this)(4,1) = (- 0.278 *  s_q_joint2_);
    (*this)(4,3) =  s_q_joint2_;
    (*this)(4,4) =  c_q_joint2_;
    (*this)(5,0) = ( 0.278 *  s_q_joint2_);
    (*this)(5,1) = ( 0.278 *  c_q_joint2_);
    (*this)(5,3) = - c_q_joint2_;
    (*this)(5,4) =  s_q_joint2_;
    return *this;
}

template <typename TRAIT>
iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link::Type_fr_arm1_X_fr_base_link()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link& iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link::update(const JState& q) {
    Scalar s_q_joint1_;
    Scalar c_q_joint1_;
    
    s_q_joint1_ = TRAIT::sin( q(JOINT1));
    c_q_joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,0) =  c_q_joint1_;
    (*this)(0,1) =  s_q_joint1_;
    (*this)(0,3) = (- 0.37 *  s_q_joint1_);
    (*this)(0,4) = ( 0.37 *  c_q_joint1_);
    (*this)(1,0) = - s_q_joint1_;
    (*this)(1,1) =  c_q_joint1_;
    (*this)(1,3) = (- 0.37 *  c_q_joint1_);
    (*this)(1,4) = (- 0.37 *  s_q_joint1_);
    (*this)(3,3) =  c_q_joint1_;
    (*this)(3,4) =  s_q_joint1_;
    (*this)(4,3) = - s_q_joint1_;
    (*this)(4,4) =  c_q_joint1_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1::Type_fr_base_link_X_fr_arm1()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1& iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1::update(const JState& q) {
    Scalar s_q_joint1_;
    Scalar c_q_joint1_;
    
    s_q_joint1_ = TRAIT::sin( q(JOINT1));
    c_q_joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,0) =  c_q_joint1_;
    (*this)(0,1) = - s_q_joint1_;
    (*this)(0,3) = (- 0.37 *  s_q_joint1_);
    (*this)(0,4) = (- 0.37 *  c_q_joint1_);
    (*this)(1,0) =  s_q_joint1_;
    (*this)(1,1) =  c_q_joint1_;
    (*this)(1,3) = ( 0.37 *  c_q_joint1_);
    (*this)(1,4) = (- 0.37 *  s_q_joint1_);
    (*this)(3,3) =  c_q_joint1_;
    (*this)(3,4) = - s_q_joint1_;
    (*this)(4,3) =  s_q_joint1_;
    (*this)(4,4) =  c_q_joint1_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1::Type_fr_arm2_X_fr_arm1()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1& iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1::update(const JState& q) {
    Scalar s_q_joint2_;
    Scalar c_q_joint2_;
    
    s_q_joint2_ = TRAIT::sin( q(JOINT2));
    c_q_joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) =  s_q_joint2_;
    (*this)(0,2) = - c_q_joint2_;
    (*this)(0,4) = ( 0.278 *  c_q_joint2_);
    (*this)(0,5) = ( 0.278 *  s_q_joint2_);
    (*this)(1,1) =  c_q_joint2_;
    (*this)(1,2) =  s_q_joint2_;
    (*this)(1,4) = (- 0.278 *  s_q_joint2_);
    (*this)(1,5) = ( 0.278 *  c_q_joint2_);
    (*this)(3,4) =  s_q_joint2_;
    (*this)(3,5) = - c_q_joint2_;
    (*this)(4,4) =  c_q_joint2_;
    (*this)(4,5) =  s_q_joint2_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2::Type_fr_arm1_X_fr_arm2()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2& iit::FurutaPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2::update(const JState& q) {
    Scalar s_q_joint2_;
    Scalar c_q_joint2_;
    
    s_q_joint2_ = TRAIT::sin( q(JOINT2));
    c_q_joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(1,0) =  s_q_joint2_;
    (*this)(1,1) =  c_q_joint2_;
    (*this)(1,3) = ( 0.278 *  c_q_joint2_);
    (*this)(1,4) = (- 0.278 *  s_q_joint2_);
    (*this)(2,0) = - c_q_joint2_;
    (*this)(2,1) =  s_q_joint2_;
    (*this)(2,3) = ( 0.278 *  s_q_joint2_);
    (*this)(2,4) = ( 0.278 *  c_q_joint2_);
    (*this)(4,3) =  s_q_joint2_;
    (*this)(4,4) =  c_q_joint2_;
    (*this)(5,3) = - c_q_joint2_;
    (*this)(5,4) =  s_q_joint2_;
    return *this;
}

template <typename TRAIT>
iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link::Type_fr_arm1_X_fr_base_link()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = - 0.37;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link& iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm1_X_fr_base_link::update(const JState& q) {
    Scalar s_q_joint1_;
    Scalar c_q_joint1_;
    
    s_q_joint1_ = TRAIT::sin( q(JOINT1));
    c_q_joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,0) =  c_q_joint1_;
    (*this)(0,1) =  s_q_joint1_;
    (*this)(1,0) = - s_q_joint1_;
    (*this)(1,1) =  c_q_joint1_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1::Type_fr_base_link_X_fr_arm1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.37;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1& iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_arm1::update(const JState& q) {
    Scalar s_q_joint1_;
    Scalar c_q_joint1_;
    
    s_q_joint1_ = TRAIT::sin( q(JOINT1));
    c_q_joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,0) =  c_q_joint1_;
    (*this)(0,1) = - s_q_joint1_;
    (*this)(1,0) =  s_q_joint1_;
    (*this)(1,1) =  c_q_joint1_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1::Type_fr_arm2_X_fr_arm1()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.278;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1& iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm2_X_fr_arm1::update(const JState& q) {
    Scalar s_q_joint2_;
    Scalar c_q_joint2_;
    
    s_q_joint2_ = TRAIT::sin( q(JOINT2));
    c_q_joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) =  s_q_joint2_;
    (*this)(0,2) = - c_q_joint2_;
    (*this)(1,1) =  c_q_joint2_;
    (*this)(1,2) =  s_q_joint2_;
    return *this;
}
template <typename TRAIT>
iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2::Type_fr_arm1_X_fr_arm2()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.278;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2& iit::FurutaPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_arm1_X_fr_arm2::update(const JState& q) {
    Scalar s_q_joint2_;
    Scalar c_q_joint2_;
    
    s_q_joint2_ = TRAIT::sin( q(JOINT2));
    c_q_joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(1,0) =  s_q_joint2_;
    (*this)(1,1) =  c_q_joint2_;
    (*this)(2,0) = - c_q_joint2_;
    (*this)(2,1) =  s_q_joint2_;
    return *this;
}

