#include "base/distorter.h"

namespace mvgplus {

void Distorter::DistortPoint(const Point2D& point2d, Point2D* distorted_point2d) {
  const Eigen::Vector2d& point = point2d.XY();
  DistorPoint(point, &(distorted_point2d->XY()));
}

void Distorter::UndistortPoint(const Point2D& distorted_point2d, Point2D* point2d) {
  const Eigen::Vector2d& distorted_point = distorted_point2d.XY();
  UndistortPoint(distorted_point2d, &(point2d->XY()));
}

void Distorter::UndistortPoint(const Eigen::Vector2d& distorted_point2d,
                               Eigen::Vector2d* point2d) {
  // Parameters for Newton iteration using numerical differentiation with
  // central differences, 100 iterations should be enough even for complex
  // camera models with higher order terms.
  const size_t kNumIterations = 100;
  const double kMaxStepNorm = 1e-10;
  const double kRelStepSize = 1e-6;

  Eigen::Matrix2d J;
  const Eigen::Vector2d x0(*u, *v);
  Eigen::Vector2d x = distorted_point2d;
  Eigen::Vector2d dx;
  Eigen::Vector2d dx_0b;
  Eigen::Vector2d dx_0f;
  Eigen::Vector2d dx_1b;
  Eigen::Vector2d dx_1f;

  for (size_t i = 0; i < kNumIterations; ++i) {
    const double step0 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(0)));
    const double step1 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(1)));
    DistortPoint(x, &dx);
    DistortPoint(x + Eigen::Vector2d(-step0, 0), &dx_0b);
    DistortPoint(x + Eigen::Vector2d(step0, 0),  &dx_0f);
    DistortPoint(x + Eigen::Vector2d(0, -step1), &dx_1b);
    DistortPoint(x + Eigen::Vector2d(0, step1),  &dx_1f);

    J(0, 0) = 1 + (dx_0f(0) - dx_0b(0)) / (2 * step0);
    J(0, 1) = (dx_1f(0) - dx_1b(0)) / (2 * step1);
    J(1, 0) = (dx_0f(1) - dx_0b(1)) / (2 * step0);
    J(1, 1) = 1 + (dx_1f(1) - dx_1b(1)) / (2 * step1);
    const Eigen::Vector2d step_x = J.inverse() * (x + dx - x0);
    x -= step_x;
    if (step_x.squaredNorm() < kMaxStepNorm) {
      break;
    }
  }

  (*point2d)[0] = x[0];
  (*point2d)[1] = x[1];
}

}  // namespace mvgplus
