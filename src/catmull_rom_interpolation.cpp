#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  int idx = -1;
  for (int i = 0; i < keyframes.size(); i++) {
	if (t == keyframes[i].first) {
	  return keyframes[i].second;
	}
	if (t < keyframes[i].first) {
	  idx = i;
	  break;
	}
  }
  if (idx <= 0) return Eigen::Vector3d(0,0,0);
  t = (t-keyframes[idx-1].first)/(keyframes[idx].first-keyframes[idx-1].first);
  Eigen::Vector3d m0, m1;
  m0 = (keyframes[idx].second-keyframes[idx-1].second)/(keyframes[idx].first-keyframes[idx-1].first);
  if (idx < keyframes.size() - 1) {
	m1 = (keyframes[idx+1].second-keyframes[idx].second)/(keyframes[idx+1].first-keyframes[idx].first);
  }else {
	m1 = m0;
  }
  return (2*t*t*t-3*t*t+1)*keyframes[idx-1].second+(t*t*t-2*t*t+t)*m0+(-2*t*t*t+3*t*t)* keyframes[idx].second+(t*t*t-t*t)*m1;
}
