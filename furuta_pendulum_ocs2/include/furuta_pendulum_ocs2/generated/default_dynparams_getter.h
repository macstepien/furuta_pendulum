#ifndef _FURUTAPENDULUM_DEFAULT_GETTER_INERTIA_PARAMETERS_
#define _FURUTAPENDULUM_DEFAULT_GETTER_INERTIA_PARAMETERS_

#include "dynamics_parameters.h"

namespace iit
{
namespace FurutaPendulum
{
namespace dyn
{

class DefaultParamsGetter : public RuntimeParamsGetter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DefaultParamsGetter() { resetDefaults(); }
  ~DefaultParamsGetter(){};

public:
  void resetDefaults() {}

private:
  RuntimeInertiaParams values;
};

}  // namespace dyn
}  // namespace FurutaPendulum
}  // namespace iit
#endif
