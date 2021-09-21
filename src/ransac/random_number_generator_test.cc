#include "ransac/random_number_generator.h"

#include <vector>
#include <gtest/gtest.h>

namespace mvgplus {

TEST(RANDOM_NUMBER_GENERATOR_TEST, TestRepeatability) {
  RandomNumberGenerator rng1;
  std::vector<size_t> numbers1;
  for (size_t i = 0; i < 100; i++) {
    numbers1.push_back(rng1.RandInt(0, 10000));
  }

  RandomNumberGenerator rng2;
  std::vector<size_t> numbers2;
  for (size_t i = 0; i < 100; i++) {
    numbers2.push_back(rng2.RandInt(0, 10000));
  }

  bool all_equal = true;
  for (size_t i = 0; i < numbers1.size(); i++) {
    if (numbers1[i] != numbers2[i]) {
      all_equal = false;
    }
  }
  
  EXPECT_EQ(all_equal, false);
}

TEST(RANDOM_NUMBER_GENERATOR_TEST, TestRandInt) {
  RandomNumberGenerator rng;
  for (size_t i = 0; i < 1000; i++) {
    EXPECT_GE(rng.RandInt(0, 100), 0);
    EXPECT_LE(rng.RandInt(0, 100), 100);
  }
}

TEST(RANDOM_NUMBER_GENERATOR_TEST, TestShuffleNone) {
  RandomNumberGenerator rng;
  std::vector<size_t> numbers(0);
  rng.Shuffle(0, &numbers);
  numbers = {1, 2, 3, 4, 5, 6};

  std::vector<size_t> shuffle_numbers = numbers;
  rng.Shuffle(0, &shuffle_numbers);
  for (size_t i = 0; i < numbers.size(); i++) {
    EXPECT_EQ(shuffle_numbers[i], numbers[i]);
  }
}

TEST(RANDOM_NUMBER_GENERATOR_TEST, TestShuffleAll) {
  std::vector<size_t> numbers(1000);
  std::iota(numbers.begin(), numbers.end(), 0);
  std::vector<size_t> shuffled_numbers = numbers;
  RandomNumberGenerator rng;
  rng.Shuffle(1000, &shuffled_numbers);
  size_t num_shuffled = 0;
  for (size_t i = 0; i < numbers.size(); ++i) {
    if (numbers[i] != shuffled_numbers[i]) {
      num_shuffled += 1;
    }
  }
  EXPECT_GT(num_shuffled, 0);
}

}  // namespace mvgplus