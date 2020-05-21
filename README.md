# Extended Kalman Filter Project
---
The goals / steps of this project are the following:
* Implement linear and non-linear Kalman filter equations
* Develop the Initialization -> Predict -> Update -> Predict... cycle
* Develop an understanding of sensor fusion through usage of both lidar and radar data
* Use the simulator to test Extended Kalman Filter algorithms
* Summarize the results with a written report


[//]: # (Image References)

[image1]: report_images/Sim1.JPG
[image2]: report_images/Sim2.JPG

---
## Project Overview
This project utilizes a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

* `main.cpp` - communicates with the simulator, receives data measurements and calls the Kalman filter and RMSE functions
* `tools.cpp` - functions to calculat ethe Jacobian matrix and RMSE
* `FusionEKF.cpp` - initializes the filter, calls the predict and update functions in the Kalman filter
* `kalman_filter.cpp` - defines the predict and update functions (lidar and radar)
* `measurement_package.h` - object to define the sensor data source and  contains the raw measurement data

My work involved the `tools.cpp`, `FusionEKF.cpp`, and `kalman_filter.cpp`.

An example of the output is shown below:

Full test:

![][image1]

Zoomed in:

![][image2]

The root-mean-squared errors were below 0.1m and 0.5m/s for x and y positions and velocities respectively.

---
## Try it yourself!

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Build pre-requisites
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

## Other Important Dependencies

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
