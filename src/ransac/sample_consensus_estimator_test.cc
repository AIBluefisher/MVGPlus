#include "ransac/sample_consensus_estimator.h"
#include "ransac/random_number_generator.h"
#include "geometry/line2d_estimator.h"

#include <gtest/gtest.h>

namespace mvgplus {

TEST(TEST_SAMPLE_CONSENSUS_ESTIMATOR, TestLineFitting) {
  RandomNumberGenerator rng;

  // Create a set of points along y=x with a small random perturbation.
  std::vector<Point2D> input_points;
  for (int i = 0; i < 10000; i++) {
    if (i % 2 == 0) {
      double noise_x = rng.RandGaussian(0.0, 0.1);
      double noise_y = rng.RandGaussian(0.0, 0.1);
      input_points.push_back(Point2D(i + noise_x, i + noise_y));
    } else {
      double noise_x = rng.RandDouble(0.0, 10000);
      double noise_y = rng.RandDouble(0.0, 10000);
      input_points.push_back(Point2D(noise_x, noise_y));
    }
  }

  RansacOptions options;
  options.error_threshold = 0.5;
  SampleConsensusEstimator<Line2DEstimator> line_consensus_estimator(
      options, input_points.size());
  Line2D line;
  RansacSummary summary;
  line_consensus_estimator.Estimate(input_points, &summary, &line);

  EXPECT_LT(std::fabs(-line.A / line.B - 1.0), 0.1);
  EXPECT_GE(summary.inlier_indices.size(), 2500);
}

}  // namespace mvgplus
