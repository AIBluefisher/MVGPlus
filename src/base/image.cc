#include "base/image.h"

namespace mvgplus {

Eigen::Matrix3d Image::RotationMatrix() const {
  Eigen::Matrix3d rotation;
}

Eigen::Vector3d Image::AngleAxis() const {
  Eigen::Vector3d angle_axis;
}

Eigen::Vector3d Image::CameraCenter() const {
  const Eigen::Matrix3d R = RotationMatrix();
  return -R * tvec_;
}


}  // namespace mvgplus
