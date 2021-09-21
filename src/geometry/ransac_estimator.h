#ifndef SRC_GEOMETRY_RANSAC_ESTIMATOR_H_
#define SRC_GEOMETRY_RANSAC_ESTIMATOR_H_

#include <vector>
#include <iostream>

namespace mvgplus {

template <class T1, class T2>
class RansacEstimator {
 public:
  typedef T1 DatumType;
  typedef T2 ModelType;

  RansacEstimator() {};
  virtual ~RansacEstimator() {}

  virtual bool ComputeModel(const std::vector<DatumType>& data,
                            std::vector<ModelType>* models) = 0;

  virtual size_t SampleSize() const = 0;

  virtual double ComputeResidual(const DatumType& data,
                                 const ModelType& model) const = 0;
};

}  // namespace mvgplus

#endif  // SRC_GEOMETRY_RANSAC_ESTIMATOR_H_
