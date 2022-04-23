#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>
#include <cmath>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  f = [&](const Eigen::VectorXd & A)->double
  {
    Skeleton copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd tips_pos = transformed_tips(copy, b);
    return (tips_pos-xb0).squaredNorm();
  };
  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Skeleton copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd tips_pos = transformed_tips(copy, b);
    Eigen::MatrixXd J;
    kinematics_jacobian(copy, b, J);
    return J.transpose()*2*(tips_pos-xb0);
  };
  proj_z = [&](Eigen::VectorXd & A)
  {
    assert(skeleton.size() * 3 == A.size());
    for (int i = 0; i < A.size(); i += 3) {
      A[i] = fmax(fmin(A[i], skeleton[i/3].xzx_max[0]), skeleton[i/3].xzx_min[0]);
      A[i+1] = fmax(fmin(A[i+1], skeleton[i/3].xzx_max[1]), skeleton[i/3].xzx_min[1]);
      A[i+2] = fmax(fmin(A[i+2], skeleton[i/3].xzx_max[2]), skeleton[i/3].xzx_min[2]);
    }
  };
}
