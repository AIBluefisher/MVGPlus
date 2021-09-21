#ifndef SRC_RANSAC_RANDOM_NUMBER_GENERATOR_H_
#define SRC_RANSAC_RANDOM_NUMBER_GENERATOR_H_

#include <thread>
#include <limits>
#include <vector>
#include <chrono>
#include <random>

namespace mvgplus {

#ifdef MVGPLUS_HAS_THREAD_LOCAL_KEYWORD
thread_local std::mt19937 rng;
#else
static std::mt19937 rng;
#endif

class RandomNumberGenerator {
 public:
  RandomNumberGenerator();

  size_t RandInt(const size_t min_num, const size_t max_num);

  double RandDouble(const double min_num, const double max_num);

  template <typename T>
  T RandGaussian(const T mean, const T std_dev);

  void Shuffle(const size_t sample_num, std::vector<size_t>* indices);

};

template <typename T>
T RandomNumberGenerator::RandGaussian(const T mean, const T std_dev) {
  std::normal_distribution<T> distribution(mean, std_dev);
  return distribution(rng);
}

}  // namespace mvgplus

#endif  // SRC_RANSAC_RANDOM_NUMBER_GENERATOR_H_
