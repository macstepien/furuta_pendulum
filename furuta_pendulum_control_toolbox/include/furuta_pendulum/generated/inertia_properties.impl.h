template <typename TRAIT>
iit::FurutaPendulum::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
  com_arm1 = iit::rbd::Vector3d(0.15, 0.0, 0.0).cast<Scalar>();
  tensor_arm1.fill(
    Scalar(0.3), com_arm1,
    rbd::Utils::buildInertiaTensor(
      Scalar(1.0E-6), Scalar(0.03155), Scalar(0.03155), Scalar(0.0), Scalar(0.0), Scalar(0.0)));

  com_arm2 = iit::rbd::Vector3d(0.148, -5.293956E-23, 0.0).cast<Scalar>();
  tensor_arm2.fill(
    Scalar(0.075), com_arm2,
    rbd::Utils::buildInertiaTensor(
      Scalar(1.0E-6), Scalar(0.0055028005), Scalar(0.0055028005), Scalar(-1.4746672E-17),
      Scalar(1.6868225E-10), Scalar(5.489853E-25)));
}
