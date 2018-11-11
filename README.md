# CarND-P08-KidnappedVehicle

CarND-P08-KidnappedVehicle solves the localization problem for a kidnapped
vehicle.  The vehicle (or its artificial intelligent system) has a map of
the location, and it is equipped with devices that provide a noisy GPS data,
noisy sensor measurements of nearby landmarks, and vehicle control data.

This project implements a particle filter algorithm to solve the vehicle
localization problem.

## File Structure
### C++ Source Files - /src
- **[main.cpp](src/main.cpp)** - runs WebSocket server to interact with 
    the Term 2 simulator.  It receives sensor and control data from the
    simulator, returns the estimated vehicle location and heading.
- **[particle_filter.cpp](src/particle_filter.cpp)** - implements 
    particle filter algorithm; `prediction`, `updateWeights`, and `resample`.
- **[map.h](src/map.h)** - implements data structure representing landmarks
    in the map.
- **[helper_functions.h](src/helper_functions.h)** - implements various
    utility data structures and functions.
### Other Support Files
- **[clean.sh](clean.sh)** - a script to clean build files
- **[build.sh](clean.sh)** - a script to build the project
- **[run.sh](clean.sh)** - a script to run the project after the build
- **[CMakeLists.txt](CMakeLists.txt)** - CMake file
- **[data/map_data.txt](data/map_data.txt)** - a map data file containing
    landmark coordinates

## Getting Started
### [Download ZIP](https://github.com/gabeoh/CarND-P08-KidnappedVehicle/archive/master.zip) or Git Clone
```
git clone https://github.com/gabeoh/CarND-P08-KidnappedVehicle.git
```

### Install uWebSockets
Run the installation script from the project repository. 
- [uWebSockets](https://github.com/uNetworking/uWebSockets)
#### Linux Installation
```
./install-ubuntu.sh
```
#### Mac Installation
```
./install-mac.sh
```

### Download Simulator
- [Udacity Self-Driving Car - Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases/)

### Dependencies
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

### Build and Run
1. Make a build directory: `mkdir build && cd build`
1. Generate Makefile: `cmake ..`
1. Compile: `make` 
1. Run it: `./particle_filter`

Alternatively, you can run provided scripts from the top-level project
directory:
1. `./clean.sh`
1. `./build.sh`
1. `./run.sh`

## Project Configuration
### Design Parameters
* **Number of particles**: The number of particles is set to 10, which
  achieved accuracy requirement within the given time limit of 100 seconds.
  This parameter can be altered to satisfy varying requirements.  The more
  the number of particles, the better accuracy is achieved with the
  sacrifice of the performance, and vice versa.

## License
Licensed under [MIT](LICENSE) License.





