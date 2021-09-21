#ifndef SRC_GEOMETRY_LINE2D_ESTIMATOR_H_
#define SRC_GEOMETRY_LINE2D_ESTIMATOR_H_

#include "geometry/ransac_estimator.h"
#include "base/point2d.h"
#include "base/line2d.h"

namespace mvgplus {

class Line2DEstimator : public RansacEstimator<Point2D, Line2D> {
 public:
  Line2DEstimator();
  virtual ~Line2DEstimator();

  bool ComputeModel(const std::vector<Point2D>& data,
                    std::vector<Line2D>* models) override;
  
  size_t SampleSize() const override;

  double ComputeResidual(const Point2D& data,
                         const Line2D& model) const override;
};

}  // namespace mvgplus

#endif  // SRC_GEOMETRY_LINE_ESTIMATOR_H_
