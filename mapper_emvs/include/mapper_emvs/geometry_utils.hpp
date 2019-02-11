#pragma once

#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>
#include <glog/logging.h>

namespace geometry_utils {

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;

typedef float Depth;
typedef float InverseDepth;
typedef Eigen::Vector3d BearingVector;
typedef Eigen::Vector2d Keypoint; // subpixel pixel coordinate
typedef Eigen::Vector3d Point;
typedef Eigen::Vector2d MotionVector;
typedef Eigen::Vector2d GradientVector;
typedef Eigen::Vector3d LinearVelocity;
typedef Eigen::Vector3d AngularVelocity;


class PinholeCamera
{

public:
  PinholeCamera() {}
  PinholeCamera(int width, int height, float fx, float fy, float cx, float cy)
    : width_(width),
      height_(height),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy)
  {
    CHECK_GT(width_, 0);
    CHECK_GT(height_, 0);
    CHECK_GT(fx_, 0.0);
    CHECK_GT(fy_, 0.0);
    CHECK_GT(cx_, 0.0);
    CHECK_GT(cy_, 0.0);

    K_ << (float) fx_, 0.f          , (float) cx_,
          0.f        , (float) fy_  , (float) cy_,
          0.f        , 0.f          ,  1.f;

    Kinv_ = K_.inverse();
  }

  inline Keypoint project3dToPixel(const Point& P)
  {
    CHECK_GE(std::fabs(P[2]), 1e-6);
    return Keypoint(fx_*P[0]/P[2] + cx_, fy_*P[1]/P[2] + cy_);
  }

  inline BearingVector projectPixelTo3dRay(const Keypoint& u)
  {
    return BearingVector((u[0]-cx_)/fx_, (u[1]-cy_)/fy_, 1.0);
  }

  Eigen::Matrix3f K() const
  {
    return K_;
  }

  int width_;
  int height_;
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  Eigen::Matrix3f K_;
  Eigen::Matrix3f Kinv_;
};

}
