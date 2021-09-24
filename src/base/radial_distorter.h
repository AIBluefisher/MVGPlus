#ifndef SRC_BASE_PINHOLE_DISTORTER_H_
#define SRC_BASE_PINHOLE_DISTORTER_H_

#include "base/distorter.h"

namespace mvgplus {

// A class which does the traditional radial distortion model.
// The number of distortion coefficients could be any number(but
// recommend to be 1~3).
class RadialDistorter : public Distorter {
 public:
  RadialDistorter() = default;

  RadialDistorter(const Distorter::DistortionType& distortion_type,
                  const std::vector<double>& distortion_params);

  virtual ~RadialDistorter() {}

  void DistortPoint(const Eigen::Vector2d& point2d,
                    Eigen::Vector2d* distorted_point2d) override;
};

}  // namespace mvgplus

#endif  // SRC_BASE_PINHOLE_DISTORTER_H_
