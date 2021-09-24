#include "base/camera.h"

namespace mvgplus {

template <class DistorterType>
Camera<DistorterType>::Camera() 
  : distorter_(nullptr),
    width_(0),
    height_(0),
    intrinsics_(std::vector<double>(5, 0)) {}

template <class DistorterType>
Camera<DistorterType>::Camera(std::unique_ptr<DistorterType>& distorter)
  : Camera() {
  distorter_ = std::move(distorter);
}

template <class DistorterType>
const double Camera<DistorterType>::Width() const {
  return width_;
}

template <class DistorterType>
const double Camera<DistorterType>::Height() const {
  return height_;
}

template <class DistorterType>
void Camera<DistorterType>::SetWidth(const double width) {
  width_ = width;
}

template <class DistorterType>
void Camera<DistorterType>::SetHeight(const double height) {
  height_ = height;
}

template <class DistorterType>
const double Camera<DistorterType>::FocalLengthX() const {
  return intrinsics_[IntrinsicsIndex::fx];
}

template <class DistorterType>
double& Camera<DistorterType>::FocalLengthXMutable() {
  return intrinsics_[IntrinsicsIndex::fx];
}

template <class DistorterType>
const double Camera<DistorterType>::FocalLengthY() const {
  return intrinsics_[IntrinsicsIndex::fy];
}

template <class DistorterType>
double& Camera<DistorterType>::FocalLengthYMutable() const {
  return intrinsics_[IntrinsicsIndex::fy];
}

template <class DistorterType>
const double Camera<DistorterType>::PrinciplePointX() const {
  return intrinsics_[IntrinsicsIndex::px];
}

template <class DistorterType>
double& Camera<DistorterType>::PrinciplePointXMutable() {
  return intrinsics_[IntrinsicsIndex::px];
}

template <class DistorterType>
const double Camera<DistorterType>::PrinciplePointY() const {
  return intrinsics_[IntrinsicsIndex::py];
}

template <class DistorterType>
double& Camera<DistorterType>::PrinciplePointYMutable() {
  return intrinsics_[IntrinsicsIndex::py];
}

template <class DistorterType>
const double Camera<DistorterType>::Skew() const {
  return intrinsics_[IntrinsicsIndex::skew];
}

template <class DistorterType>
double& Camera<DistorterType>::Skew() {
  return intrinsics_[IntrinsicsIndex::skew];
}

template <class DistorterType>
DistorterType* Camera<DistorterType>::DistorterMutable() {
  return distorter_.get();
}

template <class DistorterType>
const DistorterType& Camera<DistorterType>::GetDistorter() const {
  return distorter_.get();
}

template <class DistorterType>
void Camera<DistorterType>::ImageToWorld(const Point2D& point2d,
                                         Point2D* point2d_world) {
  const Eigen::Vector2d& point2d_uv = point2d.XY();
  Eigen::Vector2d xy;
  ImageToWorld(point2d_uv, &xy);
  (*point2d).XY() = xy;
}

template <class DistorterType>
void Camera<DistorterType>::ImageToWorld(const Eigen::Vector2d& uv,
                                         Eigen::Vector2d* xy) {
  const double fx = FocalLengthX();
  const double fy = FocalLengthY();
  const double px = PrinciplePointX();
  const double py = PrinciplePointY();

  // Lift the point to the normlized image plane.
  Eigen::Vector2d xy_distortion;
  xy_distortion[0] = (uv[0] - px) / fx;
  xy_distortion[1] = (uv[1] - py) / fy;

  // Undistort the point.
  if (distorter_ != nullptr) {
    distorter_->UndistortPoint(xy_distortion, xy);
  }
}

template <class DistorterType>
void Camera<DistorterType>::WorldToImage(const Point2D& point2d_world,
                                         Point2D* point2d) {
  const Eigen::Vector2d& point_xy = point2d_world.XY();
  Eigen::Vector2d uv;
  WorldToImage(point_xy, &uv);
  (*point2d).XY() = uv;
}

template <class DistorterType>
void Camera<DistorterType>::WorldToImage(const Eigen::Vector2d& xy,
                                         Eigen::Vector2d* uv) {
  const double fx = FocalLengthX();
  const double fy = FocalLengthY();
  const double px = PrinciplePointX();
  const double py = PrinciplePointY();

  // Distort(optionally) the point.
  Eigen::Vector2d distorted_xy = xy;
  if (distorter_ != nullptr) {
    Eigen::Vector2d delta_xy;
    distorter_->DistortPoint(xy, &delta_xy);
    distorted_xy[0] += delta_xy[0];
    distorted_xy[1] += delta_xy[1];
  }

  (*uv)[0] = fx * distorted_xy[0] + px;
  (*uv)[1] = fy * distorted_xy[1] + py;
}

}  // namespace mvgplus
