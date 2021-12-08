# Lidar obstacle detection project

## Software requirements

* PCL (POint Cloud Library) - v1.7.2
* C++ v11+
* gcc v5.5+
* cmake v3.21.4+
* Qt v.5.1

## Build instructions

## Ubuntu

   Execute the following commands in a terminal:

   ```sh
   git clone https://github.com/otakot/sensor-fusion-engineer-nd.git
   cd sensor-fusion-engineer-nd/
   sudo apt install libpcl-dev
   sudo apt install qt5-default
   mkdir build && cd build
   cmake ..
   cmake --build . --target lidar-obstacle-detection
   ./lidar-obstacle-detection
   ```
   
   ## Mac
   
   1. Install [homebrew](https://brew.sh/)
   2. Execute the following commands in a terminal:
   
   ```sh
   brew update
   brew tap brewsci/science
   brew install pcl
   brew install qt5
   export Qt5_DIR=/usr/local/opt/qt5/  # change to location of qt5 lib on your machine
   git clone https://github.com/otakot/sensor-fusion-engineer-nd.git
   cd sensor-fusion-engineer-nd/
   mkdir build && cd build
   cmake ..
   cmake --build . --target lidar-obstacle-detection
   ./lidar-obstacle-detection
   ```
