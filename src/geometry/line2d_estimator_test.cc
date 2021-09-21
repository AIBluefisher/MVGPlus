#include "geometry/line2d_estimator.h"
#include "base/line2d.h"

#include <gtest/gtest.h>

namespace mvgplus {

TEST(TEST_LINE2D_ESTIMATOR, TestLineNormal) {
  Line2D line;
  line.A = 3;
  line.B = 4;
  line.C = 5;

  const double x1 = 1.0;
  const double y1 = -(line.A * x1 + line.C) / line.B;
  Point2D point1(x1, y1);

  const double y2 = -3.0;
  const double x2 = -(line.B * y2 + line.C) / line.A;
  Point2D point2(x2, y2);

  Line2DEstimator line_estimator;
  std::vector<Line2D> lines;
  bool success = line_estimator.ComputeModel({point1, point2}, &lines);

  EXPECT_EQ(success, true);

  Line2D estimated_line = lines[0];

  EXPECT_LE(std::fabs(line.B / line.A - estimated_line.B / estimated_line.A),
            1e-8);
  EXPECT_LE(std::fabs(line.B / line.C - estimated_line.B / estimated_line.C),
            1e-8);
}

TEST(TEST_LINE2D_ESTIMATOR, TestLinePerpendicularXAxis) {
  Line2D line;
  line.A = 1;
  line.B = 0;
  line.C = 5;

  const double x1 = -5;
  const double y1 = 2;
  Point2D point1(x1, y1);

  const double x2 = -5;
  const double y2 = -3.0;
  Point2D point2(x2, y2);

  Line2DEstimator line_estimator;
  std::vector<Line2D> lines;
  bool success = line_estimator.ComputeModel({point1, point2}, &lines);

  EXPECT_EQ(success, true);

  Line2D estimated_line = lines[0];

  EXPECT_EQ(line.B, estimated_line.B);
  EXPECT_LE(std::fabs(line.A - estimated_line.A), 1e-8);
  EXPECT_LE(std::fabs(line.C - estimated_line.C), 1e-8);
}

TEST(TEST_LINE2D_ESTIMATOR, TestLinePerpendicularYAxis) {
  Line2D line;
  line.A = 0;
  line.B = 1;
  line.C = 5;

  const double y1 = -5;
  const double x1 = 2;
  Point2D point1(x1, y1);

  const double y2 = -5;
  const double x2 = -3.0;
  Point2D point2(x2, y2);

  Line2DEstimator line_estimator;
  std::vector<Line2D> lines;
  bool success = line_estimator.ComputeModel({point1, point2}, &lines);

  EXPECT_EQ(success, true);

  Line2D estimated_line = lines[0];

  EXPECT_EQ(line.A, estimated_line.A);
  EXPECT_LE(std::fabs(line.B - estimated_line.B), 1e-8);
  EXPECT_LE(std::fabs(line.C - estimated_line.C), 1e-8);
}

TEST(TEST_LINE2D_ESTIMATOR, TestSamePoints) {
  Line2D line;
  line.A = 3;
  line.B = 4;
  line.C = 5;

  const double x1 = 1.0;
  const double y1 = -(line.A * x1 + line.C) / line.B;
  Point2D point1(x1, y1);
  Point2D point2(x1 + 1e-9, y1 - 1e-9);

  Line2DEstimator line_estimator;
  std::vector<Line2D> lines;
  bool success = line_estimator.ComputeModel({point1, point2}, &lines);

  EXPECT_EQ(success, false);
}

}  // namespace mvgplus
