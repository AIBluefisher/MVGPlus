#ifndef SRC_BASE_CAMERA_H_
#define SRC_BASE_CAMERA_H_

#include <memory>
#include <vector>

#include "base/point2d.h"

namespace mvgplus {

template <class DistorterType>
class Camera {
 public:
  typedef typename DistorterType Distorter;

  enum class IntrinsicsIndex {
    fx = 0,
    fy = 1,
    px = 2,
    py = 3,
    skew = 4
  };

  Camera();
  Camera(std::unique_ptr<DistorterType>& distorter);
  ~Camera() = default;

  inline const double Width() const;
  inline const double Height() const;

  inline void SetWidth(const double width);
  inline void SetHeight(const double height);

  inline const double FocalLengthX() const;
  inline double& FocalLengthXMutable();

  inline const double FocalLengthY() const;
  inline double& FocalLengthYMutable() const;

  inline const double PrinciplePointX() const;
  inline double& PrinciplePointXMutable();

  inline const double PrinciplePointY() const;
  inline double& PrinciplePointYMutable();

  inline const double Skew() const;
  inline double& Skew();

  inline DistorterType* DistorterMutable();
  inline const DistorterType& GetDistorter() const;

  void ImageToWorld(const Point2D& point2d, Point2D* point2d_world);
  void ImageToWorld(const Eigen::Vector2d& uv, Eigen::Vector2d* xy);

  void WorldToImage(const Point2D& point2d_world, Point2D* point2d);
  void WorldToImage(const Eigen::Vector2d& xy, Eigen::Vector2d* uv);

 private:
  std::unique_ptr<Distorter> distorter_;

  double width_;
  double height_;

  std::vector<double> intrinsics_;
};

#include "base/camera.inl"

}  // namespace mvgplus

#endif  // SRC_BASE_CAMERA_H_
