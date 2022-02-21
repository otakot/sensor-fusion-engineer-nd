# Unscented Kalman Filter Project

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

This project provides implementation of Unscented Kalman Filter for estimation of the state of multiple cars on a highway using noisy lidar and radar measurements.


## Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Build Instructions

## Ubuntu

   Execute the following commands in a terminal:

   ```sh
   git clone https://github.com/otakot/sensor-fusion-engineer-nd.git
   cd sensor-fusion-engineer-nd/unscented-kalman-filter/
   sudo apt install libpcl-dev
   sudo apt install qt5-default
   mkdir build && cd build
   cmake ..
   cmake --build . --target ukf_highway
   ./ukf_highway
   ```

## MacOS

   1. Install [homebrew](https://brew.sh/)
   2. Execute the following commands in a terminal:

   ```sh
   brew update
   brew tap brewsci/science
   brew install pcl
   brew install qt5
   export Qt5_DIR=/usr/local/opt/qt5/  # change to location of qt5 lib on your machine
   git clone https://github.com/otakot/sensor-fusion-engineer-nd.git
   cd sensor-fusion-engineer-nd/unscented-kalman-filter/
   mkdir build && cd build
   cmake ..
   cmake --build . --target ukf_highway
   ./ukf_highway
   ```