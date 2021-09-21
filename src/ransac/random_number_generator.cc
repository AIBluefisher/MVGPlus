#include "ransac/random_number_generator.h"

#include <ctime>
#include <algorithm>

namespace mvgplus {

RandomNumberGenerator::RandomNumberGenerator() {
  const unsigned seed =
    static_cast<unsigned>(
        std::chrono::system_clock::now().time_since_epoch().count());
  // rng = std::mt19937(seed);
  // srand(seed);
  rng.seed(seed);
}

size_t RandomNumberGenerator::RandInt(const size_t min_num,
                                      const size_t max_num) {
  std::uniform_int_distribution<size_t> distribution(min_num, max_num);
  return distribution(rng);
}

double RandomNumberGenerator::RandDouble(const double min_num,
                                         const double max_num) {
  std::uniform_real_distribution<double> distribution(min_num, max_num);
  return distribution(rng);
}

void RandomNumberGenerator::Shuffle(const size_t num_to_shuffle,
                                    std::vector<size_t>* samples) {
  for (size_t i = 0; i < num_to_shuffle; i++) {
    const size_t j = RandInt(i, samples->size() - 1);
    std::swap((*samples)[i], (*samples)[j]);
  }
}

}  // namespace mvgplus
