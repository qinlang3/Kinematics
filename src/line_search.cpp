#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  double step = max_step;
  double energy = f(z);
  Eigen::VectorXd diff = z-step*dz;
  proj_z(diff);
  while (f(diff) > energy) {
	step = step/2.0;
	diff = z-step*dz;
	proj_z(diff);
  }
  return step;
}
