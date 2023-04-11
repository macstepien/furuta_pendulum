

//Implementation of default constructor
template <typename TRAIT>
iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    arm2_Ic(linkInertias.getTensor_arm2())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()
#define Fcol(j) (tpl::JSIM<TRAIT>:: template block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)

template <typename TRAIT>
const typename iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>& iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_arm1_X_fr_arm2(state);
    frcTransf -> fr_base_link_X_fr_arm1(state);

    // Initializes the composite inertia tensors
    base_link_Ic = linkInertias.getTensor_base_link();
    arm1_Ic = linkInertias.getTensor_arm1();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link arm2:
    iit::rbd::transformInertia<Scalar>(arm2_Ic, frcTransf -> fr_arm1_X_fr_arm2, Ic_spare);
    arm1_Ic += Ic_spare;

    Fcol(JOINT2) = arm2_Ic.col(iit::rbd::AZ);
    DATA(JOINT2+6, JOINT2+6) = Fcol(JOINT2)(iit::rbd::AZ);

    Fcol(JOINT2) = frcTransf -> fr_arm1_X_fr_arm2 * Fcol(JOINT2);
    DATA(JOINT2+6, JOINT1+6) = F(iit::rbd::AZ,JOINT2);
    DATA(JOINT1+6, JOINT2+6) = DATA(JOINT2+6, JOINT1+6);
    Fcol(JOINT2) = frcTransf -> fr_base_link_X_fr_arm1 * Fcol(JOINT2);

    // Link arm1:
    iit::rbd::transformInertia<Scalar>(arm1_Ic, frcTransf -> fr_base_link_X_fr_arm1, Ic_spare);
    base_link_Ic += Ic_spare;

    Fcol(JOINT1) = arm1_Ic.col(iit::rbd::AZ);
    DATA(JOINT1+6, JOINT1+6) = Fcol(JOINT1)(iit::rbd::AZ);

    Fcol(JOINT1) = frcTransf -> fr_base_link_X_fr_arm1 * Fcol(JOINT1);

    // Copies the upper-right block into the lower-left block, after transposing
    JSIM<TRAIT>:: template block<2, 6>(6,0) = (JSIM<TRAIT>:: template block<6, 2>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    JSIM<TRAIT>:: template block<6,6>(0,0) = base_link_Ic;
    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint joint2, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint joint1, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
}

template <typename TRAIT>
void iit::FurutaPendulum::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
}

