

//Implementation of default constructor
template <typename TRAIT>
iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::JSIM(
  IProperties & inertiaProperties, FTransforms & forceTransforms)
: linkInertias(inertiaProperties),
  frcTransf(&forceTransforms),
  arm2_Ic(linkInertias.getTensor_arm2())
{
  //Initialize the matrix itself
  this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()

template <typename TRAIT>
const typename iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT> &
iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::update(const JointState & state)
{
  ForceVector F;

  // Precomputes only once the coordinate transforms:
  frcTransf->fr_arm1_X_fr_arm2(state);

  // Initializes the composite inertia tensors
  arm1_Ic = linkInertias.getTensor_arm1();

  // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

  // Link arm2:
  iit::rbd::transformInertia<Scalar>(arm2_Ic, frcTransf->fr_arm1_X_fr_arm2, Ic_spare);
  arm1_Ic += Ic_spare;

  F = arm2_Ic.col(iit::rbd::AZ);
  DATA(JOINT2, JOINT2) = F(iit::rbd::AZ);

  F = frcTransf->fr_arm1_X_fr_arm2 * F;
  DATA(JOINT2, JOINT1) = F(iit::rbd::AZ);
  DATA(JOINT1, JOINT2) = DATA(JOINT2, JOINT1);

  // Link arm1:

  F = arm1_Ic.col(iit::rbd::AZ);
  DATA(JOINT1, JOINT1) = F(iit::rbd::AZ);

  return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::computeL()
{
  L = this->template triangularView<Eigen::Lower>();
  // Joint joint2, index 1 :
  L(1, 1) = std::sqrt(L(1, 1));
  L(1, 0) = L(1, 0) / L(1, 1);
  L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);

  // Joint joint1, index 0 :
  L(0, 0) = std::sqrt(L(0, 0));
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::computeInverse()
{
  computeLInverse();

  inverse(0, 0) = +(Linv(0, 0) * Linv(0, 0));
  inverse(1, 1) = +(Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
  inverse(1, 0) = +(Linv(1, 0) * Linv(0, 0));
  inverse(0, 1) = inverse(1, 0);
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::computeLInverse()
{
  //assumes L has been computed already
  Linv(0, 0) = 1 / L(0, 0);
  Linv(1, 1) = 1 / L(1, 1);
  Linv(1, 0) = -Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
}
