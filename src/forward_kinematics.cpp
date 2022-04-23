#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function


void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  T.resize(skeleton.size(), Eigen::Affine3d::Identity());
  for (int i = 0; i < skeleton.size(); i++) {
	Bone curr = skeleton[i];
	Eigen::Affine3d t;
	t = Eigen::Affine3d::Identity();
	int idx = i;
	while(idx != -1) {
	  auto curr = skeleton[idx];
	  t = curr.rest_T * euler_angles_to_transform(curr.xzx) * curr.rest_T.inverse() * t;
	  idx = curr.parent_index;
	}
	T[i] = t;
  }
}
