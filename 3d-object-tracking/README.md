# 3D Object Tracking

This project provides implementation of real timne system for tracking of 3D Objects (vehicles on the road in the traffic) using Lidar and mono camera sensors installed on ego vehicle. The high level architecture of implemented system looks following:

<img src="images/course_code_structure.png" width="779" height="414" />


## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build Instructions

### Ubuntu, MacOS

   Execute the following commands in a terminal:

   ```sh
   git clone https://github.com/otakot/sensor-fusion-engineer-nd.git
   cd sensor-fusion-engineer-nd/2d-object-tracking/
   mkdir build && cd build
   cmake ..
   cmake --build . --target 3D_object_tracking
   ./3D_object_tracking
   ```
