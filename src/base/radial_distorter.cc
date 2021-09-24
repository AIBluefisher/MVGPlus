#include "base/pinhole_distorter.h"

#include <cmath>

namespace mvgplus {
RadialDistorter::RadialDistorter(const Distorter::DistortionType& distortion_type,
                   const std::vector<double>& distortion_params)
  : Distorter(distortion_type, distortion_params) {}


void RadialDistorter::DistortPoint(const Eigen::Vector2d& point2d,
                                    Eigen::Vector2d* delta_point2d) {
  const double u2 = point2d[0] * point2d[0];
  const double v2 = point2d[1] * point2d[1];
  const double r2 = u2 + v2;
  double distortion_factor = 0;
  for (size_t i = 0; i < distortion_params_.size(); i++) {
    const double ki = distortion_params_[i];
    distortion_factor += ki * std::pow(ki, i + 1);
  }

  (*delta_point2d)[0] = distortion_factor * point2d[0];
  (*delta_point2d)[1] = distortion_factor * point2d[1];
}

}  // namespace mvgplus
