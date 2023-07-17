template <typename TRAIT>
iit::FurutaPendulum::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
  com_arm1 = iit::rbd::Vector3d(0.05415, 0.0, 0.0).cast<Scalar>();
  tensor_arm1.fill(
    Scalar(0.040466), com_arm1,
    rbd::Utils::buildInertiaTensor(
      Scalar(1.0E-6), Scalar(1.4270964E-4), Scalar(1.4270964E-4), Scalar(0.0), Scalar(0.0),
      Scalar(0.0)));

  com_arm2 = iit::rbd::Vector3d(0.096608, -5.293956E-23, 0.0).cast<Scalar>();
  tensor_arm2.fill(
    Scalar(0.009801), com_arm2,
    rbd::Utils::buildInertiaTensor(
      Scalar(1.0E-6), Scalar(1.0626436E-4), Scalar(1.0626436E-4), Scalar(-5.269898E-20),
      Scalar(6.0280557E-13), Scalar(3.0726126E-26)));
}
