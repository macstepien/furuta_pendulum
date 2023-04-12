#include <ct/rbd/rbd.h>
#include <ct/core/plot/plot.h>

#include <furuta_pendulum/FurutaPendulum.h>

int main()
{
  // obtain the state dimension
  const size_t STATE_DIM =
    ct::rbd::FixBaseFDSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>::STATE_DIM;

  // create an instance of the system
  std::shared_ptr<ct::core::System<STATE_DIM>> dynamics(
    new ct::rbd::FixBaseFDSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>);

  ct::core::Integrator<STATE_DIM> integrator(dynamics, ct::core::RK4);
  ct::core::StateVector<STATE_DIM> state;
  state.setZero();
  state(0) = 0.0;
  state(1) = 2.0;
  state(2) = 0.0;
  state(3) = 0.0;

  ct::core::StateVectorArray<STATE_DIM, double> state_trajectory;
  ct::core::TimeArray time_trajectory;
  integrator.integrate_n_steps(state, 0, 1000, 0.001, state_trajectory, time_trajectory);

  std::cout << state << std::endl;

  std::vector<double> state_1;
  std::vector<double> state_2;
  std::vector<double> state_3;
  std::vector<double> state_4;
  std::vector<double> time_state;

  for (size_t j = 0; j < state_trajectory.size(); j++) {
    state_1.push_back(state_trajectory[j](0));
    state_2.push_back(state_trajectory[j](1));
    state_3.push_back(state_trajectory[j](2));
    state_4.push_back(state_trajectory[j](3));
    time_state.push_back(time_trajectory[j]);
  }

  ct::core::plot::plot(time_state, state_1, "r");  //pos 1
  ct::core::plot::plot(time_state, state_3, "g");  //vel 1

  ct::core::plot::plot(time_state, state_2, "b");  //pos 2
  ct::core::plot::plot(time_state, state_4, "c");  //vel 2

  ct::core::plot::show();
}
