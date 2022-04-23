#include "euler_angles_to_transform.h"
#define M_PI 3.1415926535897932384626433

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  Eigen::Affine3d A;
  A = Eigen::AngleAxisd(xzx[2]/180 * M_PI, Eigen::Vector3d::UnitX())
	* Eigen::AngleAxisd(xzx[1]/180 * M_PI, Eigen::Vector3d::UnitZ())
	* Eigen::AngleAxisd(xzx[0]/180 * M_PI, Eigen::Vector3d::UnitX());
  return A;
}
