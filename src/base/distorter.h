#ifndef SRC_BASE_DISTORTER_H_
#define SRC_BASE_DISTORTER_H_

#include <vector>
#include <Eigen/Core>

#include "base/point2d.h"

namespace mvgplus {

// A virtual base class which defines the distortion/unditortion actions.
// All class inherites from this class should override the `DistortPoint`
// function.
class Distorter {
 public:
  enum class DistortionType {
    INVALID = -1,
    PINHOLE = 0,
    FOV = 1
  };

  Distorter() = default;
  Distorter(const Distorter::DistortionType& distortion_type,
            const std::vector<double>& distortion_params)
    : distortion_type_(distortion_type),
      distortion_params_(distortion_params) {}

  virtual ~Distorter() = default;

  inline void SetDistortionType(const Distorter::DistortionType& distortion_type);
  inline const Distorter::DistortionType& DistortionType() const;
  inline Distorter::DistortionType& DistortionTypeMutable();

  inline void SetDistortionParams(const std::vector<double>& distortion_params);
  inline const std::vector<double>& DistortionParams() const;
  inline std::vector<double>& DistortionParamsMutable();

  void DistortPoint(const Point2D& point2d, Point2D* distorted_point2d);

  virtual void DistortPoint(const Eigen::Vector2d& point2d,
                            Eigen::Vector2d* delta_point2d) = 0;

  void UndistortPoint(const Point2D& distorted_point2d, Point2D* point2d);

  void UndistortPoint(const Eigen::Vector2d& distorted_point2d,
                              Eigen::Vector2d* point2d);

 protected:
  DistortionType distortion_type_;
  std::vector<double> distortion_params_;
};

}  // namespace mvgplus

inline void Distorter::SetDistortionType(
    const Distorter::DistortionType& distortion_type) {
  distortion_type_ = distortion_type;
}

inline const Distorter::DistortionType& Distorter::DistortionType() const {
  return distortion_type_;
}

inline Distorter::DistortionType& Distorter::DistortionTypeMutable() {
  return distortion_type_;
}

inline void Distorter::SetDistortionParams(
    const std::vector<double>& distortion_params) {
  distortion_params_ = distortion_params;
}

inline const std::vector<double>& Distorter::DistortionParams() const {
  return distortion_params_;
}

inline std::vector<double>& Distorter::DistortionParamsMutable() {
  return distortion_params_;
}

#endif  // SRC_BASE_DISTORTER_H_
