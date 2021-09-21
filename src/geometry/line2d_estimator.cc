#include "geometry/line2d_estimator.h"

#include <cmath>

namespace mvgplus {

Line2DEstimator::Line2DEstimator() : RansacEstimator() {}

Line2DEstimator::~Line2DEstimator() {}

bool Line2DEstimator::ComputeModel(const std::vector<Point2D>& data,
                                   std::vector<Line2D>* models) {
  if (data.size() != SampleSize()) {
    return false;
  }

  const Point2D& point1 = data[0];
  const Point2D& point2 = data[1];

  const double x1 = point1.X();
  const double y1 = point1.Y();

  const double x2 = point2.X();
  const double y2 = point2.Y();

  const double kSamePosition = 1e-8;

  if (std::fabs(x1 - x2) < kSamePosition &&
      std::fabs(y1 - y2) < kSamePosition) {
    return false;
  }

  Line2D model;

  if (std::fabs(x1 - x2) < kSamePosition) {
    model.A = 1.0;
    model.B = 0.0;
    model.C = -(x1 + x2) / 2.0;
  } else if (std::fabs(y1 - y2) < kSamePosition) {
    model.A = 0.0;
    model.B = 1.0;
    model.C = -(y1 + y2) / 2.0;
  } else {
    model.A = y2 - y1;
    model.B = x1 - x2;
    model.C = (x2 - x1) * y1 + (y1 - y2) * x1;
  }

  (*models).emplace_back(model);

  return true;
}

size_t Line2DEstimator::SampleSize() const {
  return 2;
}

double Line2DEstimator::ComputeResidual(const Point2D& data,
                                        const Line2D& model) const {
  const double A = model.A;
  const double B = model.B;
  const double C = model.C;

  return std::fabs(A * data.X() + B * data.Y() + C) /
          std::sqrt(A * A + B * B);
}

}  // namespace mvgplus
