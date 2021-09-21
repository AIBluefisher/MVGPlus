#ifndef SRC_RANSAC_SAMPLER_H_
#define SRC_RANSAC_SAMPLER_H_

#include <vector>
#include <memory>

#include <ransac/random_number_generator.h>

namespace mvgplus {

class Sampler {
 public:
  Sampler(const size_t num_data);

  virtual ~Sampler();

  virtual void Sample(const size_t min_num_sample,
                      std::vector<size_t>* indices);

 private:
  size_t num_data_;
  std::vector<size_t> sample_indices_;
  std::unique_ptr<RandomNumberGenerator> rng_;

};

}  // namespace mvgplus

#endif  // SRC_RANSAC_SAMPLER_H_
