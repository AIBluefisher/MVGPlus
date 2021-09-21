#ifndef SRC_RANSAC_SAMPLE_CONSENSUS_ESTIMATOR_H_
#define SRC_RANSAC_SAMPLE_CONSENSUS_ESTIMATOR_H_

#include <vector>
#include <memory>
#include <iostream>

#include "ransac/sampler.h"
#include "ransac/ransac.h"

namespace mvgplus {

struct RansacOptions {
  // The probability of at least once there is no outlier exists in the sample
  // points when we sample N times.
  double probability = 0.99;

  // Threshold indicate if the data is an inlier.
  double error_threshold = 0.0;

  // The initial guess which is used to compute the maximal iteration number.
  double min_inlier_ratio = 0.2;

  // Minimal number of ransac iterations.
  size_t min_iteration_num = 100;

  // Maximal number of ransac iterations.
  size_t max_iteration_num = 1000;
};

struct RansacSummary {
  // If ransac succeed.
  bool success = false;

  // Total number of ransac iterations.
  size_t num_iterations = 0;

  // The indices of inliers.
  std::vector<size_t> inlier_indices;
};

/////////////////////////////////////////////////////////////////////////////
// The basic class which implements a naive ransac algorithms, with
// the maximal number iterations can be computed adaptively.
// Note that we use a templated implementation of the class, as the
// geometric model is determined by a specific task and is only known by
// users.
// Usage:
//  std::vector<DataType> input_points;
//  RansacOptions options;
//  options.error_threshold = ...;
//  options.probability = ...;
//  options.min_inlier_ratio = ...;
//  SampleConsensusEstimator<EstimatorClass> consensus_estimator(
//      options, input_points.size());
//  ModelType model;
//  RansacSummary summary;
//  line_consensus_estimator.Estimate(input_points, &summary, &model);
////////////////////////////////////////////////////////////////////////////
template <class EstimatorType>
class SampleConsensusEstimator {
 public:
  typedef typename EstimatorType::ModelType model_t;
  typedef typename EstimatorType::DatumType datum_t;

  SampleConsensusEstimator(const RansacOptions& options,
                           const size_t num_data);

  // A virtual function which used to estimate the best model.
  // This function should be override when a new class inheritates from
  // class `SampleConsensusEstimator`
  virtual void Estimate(const std::vector<datum_t>& samples,
                        RansacSummary* summary,
                        model_t* model);

 private:
  const RansacOptions options_;

  std::unique_ptr<Sampler> sampler_;

  std::unique_ptr<EstimatorType> estimator_;
};

template <class EstimatorType>
SampleConsensusEstimator<EstimatorType>::SampleConsensusEstimator(
    const RansacOptions& options,
    const size_t num_data) : options_(options),
                             sampler_(new Sampler(num_data)),
                             estimator_(new EstimatorType()) {}

template <class EstimatorType>
void SampleConsensusEstimator<EstimatorType>::Estimate(
    const std::vector<datum_t>& data,
    RansacSummary* summary,
    model_t* best_model) {
  // CHECK_NOTNULL(sampler_);
  // CHECK_NOTNULL(estimator_);
  // CHECK_GE(data.size(), estimator_->SampleSize());

  const size_t min_num_samples = estimator_->SampleSize();

  // As the actual inlier ration is unknown, we can only compute the maximal
  // number of iterations from an bad initial guess.
  size_t max_num_iterations = std::min(options_.max_iteration_num,
      ComputeMaxIterationNum(min_num_samples, options_.min_inlier_ratio,
          options_.probability));

  for (size_t i = 0; i < max_num_iterations; i++) {
    // 1. Randomly sampling data points.
    std::vector<size_t> indices;
    sampler_->Sample(min_num_samples, &indices);

    std::vector<datum_t> sample;
    for (size_t k = 0; k < min_num_samples; k++) {
      sample.emplace_back(data[indices[k]]);
    }

    // 2. Computing a model by the given data.
    std::vector<model_t> models;  // estimator might compute several models.
    bool success = estimator_->ComputeModel(sample, &models);
    if (!success) {
      continue;
    }

    // 3. Finding the consensus subset.
    double inlier_ratio = 0.0;
    for (const model_t& model : models) {
      std::vector<size_t> inlier_indices;
      for (size_t i = 0; i < data.size(); i++) {
        const double error = estimator_->ComputeResidual(data[i], model);
        if (error <= options_.error_threshold) {
          inlier_indices.push_back(i);
        }
      }

      // Validating the consensus set.
      inlier_ratio = static_cast<double>(inlier_indices.size()) /
          static_cast<double>(data.size());

      if (inlier_ratio < options_.min_inlier_ratio) {
        continue;
      }

      // If the set is the maximal subset we have seen so far,
      // then we update the consensus subset.
      if (inlier_indices.size() > summary->inlier_indices.size()) {
        inlier_indices.swap(summary->inlier_indices);
        *best_model = model;
        summary->success = true;
      }
    }

    // 4. Adaptively computing the maximal iteration number.
    max_num_iterations = std::min(max_num_iterations, ComputeMaxIterationNum(
        min_num_samples, inlier_ratio, options_.probability));
  }

  summary->num_iterations = max_num_iterations;
}


}  // namespace mvgplus

#endif  // SRC_RANSAC_SAMPLE_CONSENSUS_ESTIMATOR_H_
