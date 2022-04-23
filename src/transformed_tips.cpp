#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > T;
  forward_kinematics(skeleton, T);
  Eigen::VectorXd tips_pos = Eigen::VectorXd::Zero(3 * b.size());
  for (int i = 0; i < b.size(); i++) {
	Eigen::Vector4d s = T[b[i]]*skeleton[b[i]].rest_T*Eigen::Vector4d(skeleton[b[i]].length, 0.0, 0.0, 1.0);
	tips_pos[i*3+0] = s[0];
	tips_pos[i*3+1] = s[1];
	tips_pos[i*3+2] = s[2];
  }
  return tips_pos;
}
