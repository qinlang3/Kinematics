#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>
#include "copy_skeleton_at.h"

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  double h = 1e-7;
  Skeleton copy = skeleton;
  Eigen::VectorXd tips_pos = transformed_tips(skeleton, b);
  for (int i = 0; i < skeleton.size(); i++) {
	copy[i].xzx[0] += h;
	J.col(i*3) = (transformed_tips(copy, b)-tips_pos)/h;
	copy[i].xzx[0] -= h;
	copy[i].xzx[1] += h;
	J.col(i*3+1) = (transformed_tips(copy, b)-tips_pos)/h;
	copy[i].xzx[1] -= h;
	copy[i].xzx[2] += h;
	J.col(i*3+2) = (transformed_tips(copy, b)-tips_pos)/h;
	copy[i].xzx[2] -= h;
  }
}
