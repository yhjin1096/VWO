# VWO
Visual Wheel Odometry

# g2o
### common
```
sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
```
### first method
[https://github.com/stella-cv/stella_vslam/blob/main/Dockerfile.desktop](https://github.com/stella-cv/stella_vslam/blob/21a11916aa9070c0089993172269521eae70167d/Dockerfile.desktop#L81)
```
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 20230223_git
mkdir build install
cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/home/cona/yhj/VWO/src/VWO/thirdparty/g2o/install \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=OFF \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_LINKED_APPS=OFF \
    ..
```
### second method
```
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build install
cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/cona/yhj/VWO/src/VWO/thirdparty/g2o/install ..
make j4 && make install
```

## CmakeList.txt
### first method
```
set(g2o_DIR "${PROJECT_SOURCE_DIR}/thirdparty/g2o/install/lib/cmake/g2o")
find_package(g2o REQUIRED)
```
### second method
```
find_package(g2o REQUIRED)
include_directories(
  /usr/include/suitesparse # cs.h
)
```
