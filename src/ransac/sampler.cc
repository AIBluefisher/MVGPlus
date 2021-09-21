#include "ransac/sampler.h"

#include <algorithm>
#include <memory>

namespace mvgplus {

Sampler::Sampler(const size_t num_data)
    : num_data_(num_data),
      rng_(new RandomNumberGenerator()) {
  sample_indices_.resize(num_data);
  std::iota(sample_indices_.begin(), sample_indices_.end(), 0);
  
  srand(static_cast<int>(time(0)));
}

Sampler::~Sampler() {}

void Sampler::Sample(const size_t min_num_samples,
                     std::vector<size_t>* indices) {
  rng_->Shuffle(min_num_samples, &sample_indices_);
  for (size_t i = 0; i < min_num_samples; i++) {
    (*indices).push_back(sample_indices_[i]);
  }
}

}  // namespace mvgplus
