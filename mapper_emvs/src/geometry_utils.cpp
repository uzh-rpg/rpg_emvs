#include <mapper_emvs/geometry_utils.hpp>


namespace geometry_utils {

Transformation poseTFToMinkindr(const tf::Pose& pose)
{
  Eigen::Affine3d tr;
  tf::poseTFToEigen(pose, tr);
  Eigen::Matrix4d mat = tr.matrix();
  Transformation minkindr_t(mat);
  return minkindr_t;
}

}
