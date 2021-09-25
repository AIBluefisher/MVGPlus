#include "geometry/homography2d_estimator.h"

#include <gtest/gtest.h>

namespace mvgplus {

TEST(TEST_HOMOGRAPHY_ESTIMATOR, TestComputeModel) {
  for (double x = 0; x < 10; ++x) {
    Eigen::Matrix3d H0;
    H0 << x, 0.2, 0.3, 30, 0.2, 0.1, 0.3, 20, 1;

    std::vector<Eigen::Vector2d> src;
    src.emplace_back(x, 0);
    src.emplace_back(1, 0);
    src.emplace_back(2, 1);
    src.emplace_back(10, 30);

    std::vector<Correspondence2D> corres;
    for (size_t i = 0; i < 4; ++i) {
      const Eigen::Vector3d dst = H0 * src[i].homogeneous();
      corres.emplace_back(Point2D(src[i]), Point2D(dst.hnormalized()));
    }

    Homography2DEstimator estimator;
    std::vector<Homography2D> H_ests;
    EXPECT_TRUE(estimator.ComputeModel(corres, &H_ests));

    std::cout << H_ests[0] << std::endl << std::endl;

    for (size_t i = 0; i < 4; ++i) {
      const double res = estimator.ComputeResidual(corres[i], H_ests[0]);
      EXPECT_LE(res, 1e-6);
    }
  }
}

}  // namespace mvgplus
