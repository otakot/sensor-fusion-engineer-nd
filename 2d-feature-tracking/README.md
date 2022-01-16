# 2D Feature Tracking project

<img src="images/keypoints.png" width="820" height="248" />

## Software requirements

* cmake >= 2.8
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* OpenCV >= 4.1
  ** It must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
* gcc/g++ >= 5.4
##  Build Instructions
### Ubuntu, MacOS

   Execute the following commands in a terminal:

   ```sh
   git clone https://github.com/otakot/sensor-fusion-engineer-nd.git
   cd sensor-fusion-engineer-nd/2d-feature-tracking/
   mkdir build && cd build
   cmake ..
   cmake --build . --target 2D_feature_tracking
   ./2D_feature_tracking
   ```