cmake_minimum_required(VERSION 2.8)
project(MVGPlus)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

find_package(GTest REQUIRED)
if (GTEST_FOUND)
  message("Found GTest: ${GTEST_INCLUDE_DIRS}")
  set(GTEST_LIBRARIES ${GTEST_MAIN_LIBRARY} ${GTEST_LIBRARY})
else (GTEST_FOUND)
   message(FATAL_ERROR "Can't find GTest. Please set GTEST_INCLUDE_DIRS & GTEST_LIBRARIES")
endif (GTEST_FOUND)

include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(src)

enable_testing()

add_library(${PROJECT_NAME} SHARED
  src/base/camera.h
  src/base/camera.inl
  src/base/distorter.h
  src/base/distorter.cc
  src/base/image.h
  src/bsae/image.cc
  src/base/radial_distorter.h
  src/base/radial_distorter.cc
  src/base/line2d.h
  src/base/point2d.h
  src/geometry/line2d_estimator.h
  src/geometry/line2d_estimator.cc
  src/geometry/ransac_estimator.h
  src/ransac/random_number_generator.h
  src/ransac/random_number_generator.cc
  src/ransac/ransac.h
  src/ransac/sample_consensus_estimator.h
  src/ransac/sampler.h
  src/ransac/sampler.cc
  )

#############################################################################################
#                                  Unit Tests                                               #
#############################################################################################

add_executable(line2d_estimator_test src/geometry/line2d_estimator_test.cc)
target_link_libraries(line2d_estimator_test
                      ${PROJECT_NAME} ${GTEST_LIBRARIES} -lpthread)
add_test(line2d_estimator_test
         ${EXECUTABLE_OUTPUT_PATH}/line2d_estimator_test)

add_executable(random_number_generator_test src/ransac/random_number_generator_test.cc)
target_link_libraries(random_number_generator_test
                      ${PROJECT_NAME} ${GTEST_LIBRARIES} -lpthread)
add_test(random_number_generator_test
         ${EXECUTABLE_OUTPUT_PATH}/random_number_generator_test)

add_executable(sample_consensus_estimator_test src/ransac/sample_consensus_estimator_test.cc)
target_link_libraries(sample_consensus_estimator_test
                      ${PROJECT_NAME} ${GTEST_LIBRARIES} -lpthread)
add_test(sample_consensus_estimator_test
         ${EXECUTABLE_OUTPUT_PATH}/sample_consensus_estimator_test)