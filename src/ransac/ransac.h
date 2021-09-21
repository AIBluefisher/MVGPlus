#ifndef SRC_RANSAC_RANSAC_H_
#define SRC_RANSAC_RANSAC_H_

#include <vector>
#include <memory>
#include <cmath>

namespace mvgplus {

size_t ComputeMaxIterationNum(const size_t sample_size,
                              const double inlier_ratio,
                              const double probability = 0.99) {
  return std::log(1 - probability) /
      std::log(1.0 - std::pow(inlier_ratio, sample_size));
}

}  // namespace mvgplus

#endif  // SRC_RANSAC_RANSAC_H_
