# PID Controller Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This repository contains my implementation of the PID Controller project (Term 2 - Project 4) in Udacity's Self-Driving Car Nanodegree Program.
The goal of the project is to implement a PID controller to race around the lake track in the driving simulation.


## Implementation 

One PID controller was used to control the throttle of the vehicle in order to achieve a constant speed of 20 km/s. 
The parameters of the throttle PID controller were set to: Kp=0.05, Ki = 0.005 and Kd = 0.

Another PID controller was used to control the steering. 
Manual parameter tuning was used to get a feeling for the impact of the single parameters. 

Afterwards, a twiddle algorithm was used to automatically fine tune the parameters. 
The algorithm was implemented similar to the version showin in the class. 
For every set of parameters, the simulator was first reset and then the code was run for the first 500 timestamps of the simulator.
Using this procedure, the total error of more than 200 parameter configurations was recorded.
The parameter-set which resulted in the minimum error resulted in:

This combination resulted in a total error of: .
Thus, this combination was used for the submission of the project. 

The implementation of the twiddle algorithm can be found in main_twiddle.cpp. 

## Demo

The chosen parameter configuration resulted in the following test video: 



## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid


## Dependencies
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

