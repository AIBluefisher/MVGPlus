#ifndef SRC_BASE_IMAGE_H_
#define SRC_BASE_IMAGE_H_

#include <Eigen/Core>

#include "base/point2d.h"
#include "base/point3d.h"

namespace mvgplus {

class Image {
 public:
  Image() = default;

  inline void SetQvec(const Eigen::Vector4d& qvec);
  inline const Eigen::Vector4d& Qvec() const;
  inline Eigen::Vector4d& QvecMutable();

  Eigen::Matrix3d RotationMatrix() const;
  Eigen::Vector3d AngleAxis() const;

  inline void SetTvec(const Eigen::Vector3d& tvec);
  inline const Eigen::Vector3d& Tvec() const;
  inline Eigen::Vector3d& TvecMutable() const;

  Eigen::Vector3d CameraCenter() const;

 private:
  // Camera pose which reprojects landmarks from world frame to image frame.
  Eigen::Vector4d qvec_;
  Eigen::Vector3d tvec_;
};

inline void Image::SetQvec(const Eigen::Vector4d& qvec) {
  qvec_ = qvec;
}

inline const Eigen::Vector4d& Image::Qvec() const {
  return qvec_;
}

inline Eigen::Vector4d& Image::QvecMutable() {
  return qvec_;
}

inline void Image::SetTvec(const Eigen::Vector3d& tvec) {
  tvec_ = tvec;
}

inline const Eigen::Vector3d& Image::Tvec() const {
  return tvec_;
}

inline Eigen::Vector3d& Image::TvecMutable() const {
  return tvec_;
}

}  // namespace mvgplus

#endif  // SRC_BASE_IMAGE_H_
