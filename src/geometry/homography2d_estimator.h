#ifndef SRC_GEOMETRY_HOMOGRAPHY2D_ESTIMATOR_H_
#define SRC_GEOMETRY_HOMOGRAPHY2D_ESTIMATOR_H_

#include "base/types.h"
#include "base/correspondence.h"
#include "geometry/ransac_estimator.h"

namespace mvgplus {

class Homography2DEstimator : public RansacEstimator<Correspondence2D, Homography2D> {
 public:
  Homography2DEstimator();
  virtual ~Homography2DEstimator();

  bool ComputeModel(const std::vector<Correspondence2D>& data,
                    std::vector<Homography2D>* models) override;

  size_t SampleSize() const override;

  double ComputeResidual(const Correspondence2D& data,
                         const Homography2D& model) const override;
};

}  // namespace mvgplus

#endif  // SRC_GEOMETRY_HOMOGRAPHY2D_ESTIMATOR_H_
