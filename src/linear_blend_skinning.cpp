#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  U.resize(V.rows(), V.cols());
  for (int i = 0; i < V.rows(); i++) {
	Eigen::Vector4d v = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
	for (int j = 0; j < skeleton.size(); j++) {
	  if (skeleton[j].weight_index != -1) v += W(i, skeleton[j].weight_index) * (T[j] * Eigen::Vector4d(V(i, 0), V(i, 1), V(i, 2), 1.0));
	}
	U.row(i) = Eigen::Vector3d(v[0], v[1], v[2]);
  }
}
