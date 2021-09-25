# MVGPlus

Implementations of Multiple View Geometry in Computer Vision and some extended algorithms.

## Implementations

- RANSAC
- 2D Line estimator
- Rotation Conversion (Rotation Matrix <=> Quaternion <=> Angle Axis). Some code inspired by colmap,
and some by ceres.

## Dependencies

- Eigen (3.2 or above) for matrix manipulation.
- GTest for unit tests.

### Installation
#### Ubuntu

```bash
sudo apt-get install git cmake build-essential libgtest-dev

git clone https://github.com/eigenteam/eigen-git-mirror.git
cd eigen-git-mirror
git checkout 3.2.10
mkdir build && cd build
cmake .. && make
sudo make install

cd /usr/src/gtest
sudo mkdir build && cd build
sudo cmake .. && sudo make
sudo cp libgtest* /usr/lib/
cd ../ && sudo rm -rf build
```

## Compilation

```bash
git clone https://github.com/AIBluefisher/MVGPlus.git
cd MVGPlus
mkdir build
cd build
cmake ..
make -j8
```

Note: For practice, please checking out to the `practice` branch to complete
the code, and then make sure you passed all the unit tests.
```bash
cd MVGPlus
git checkout practice
mkdir build
cd build
cmake ..&& make -j8
make test
```