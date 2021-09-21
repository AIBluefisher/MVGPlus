#ifndef SRC_BASE_POINT2D_H_
#define SRC_BASE_POINT2D_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mvgplus {

class Point2D {
 public:
  Point2D() : Point2D(0, 0) {}

  Point2D(const double x, const double y) : xy_(x, y) {}

  Point2D(const Eigen::Vector2d& point) : xy_(point) {}

  inline const double X() const { return xy_[0]; }

  inline const double Y() const { return xy_[1]; }

  inline const Eigen::Vector2d& XY() const { return xy_; }

  inline Eigen::Vector2d& XY() { return xy_; }

  inline const Eigen::Vector3d ToHomogeneous() const { return xy_.homogeneous(); }

 private:
  Eigen::Vector2d xy_;
};

}  // namespace mvgplus

#endif  // SRC_BASE_POINT2D_H_
