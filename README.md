# Kalman Filtering and Sensor Fusion Simulation #

 Kalman Filtering and Sensor Fusion Simulation exercise project. In this project, you will be developing the source code with Kalman Filters which are used to estimate the navigation state of autonomous driving.


![AKFSF-Simulation](/KalmanF.gif)


This README is broken down into the following sections:

- [cpp](#C++) - the cpp code setup running.
 - [py](#Python) - the python code base on cpp version
 
 Clone the repository
 ```
 git clone git@github.com:NIEYALI/KalmanFilter.git
 ```

 ## C++ ##

This project will use the Ubuntu 64 22.04.2.0 LTS VM C++ development environment. Please follow the steps below to compile the simulation.

 1. Install the dependencies
 ```
 sudo apt install libeigen3-dev libsdl2-dev libsdl2-ttf-dev
 ```
 

 1. Setup the cmake build
 ```
 cd cpp
 mkdir build
 cd build
 cmake ../
 ```

 1. Compile the code
 ```
 make
 ```
 
 1. You should now be able to and run the estimation simulator
 ```
 ./AKFSF-Simulation
 ```



## Python ##
 1. Install the dependencies
 ```
 pip install -r requirements.txt

 ```
 
 1. Run the estimation simulator
 ```
python main.py
```

### Motion Profiles ###
* 1 - Constant Velocity + GPS + GYRO + Zero Initial Conditions
* 2 - Constant Velocity + GPS + GYRO + Non-zero Initial Conditions
* 3 - Constant Speed Profile + GPS + GYRO
* 4 - Variable Speed Profile + GPS + GYRO
* 5 - Constant Velocity + GPS + GYRO + LIDAR+ Zero Initial Conditions
* 6 - Constant Velocity + GPS + GYRO + LIDAR + Non-zero Initial Conditions
* 7 - Constant Speed Profile + GPS + GYRO + LIDAR
* 8 - Variable Speed Profile + GPS + GYRO + LIDAR
* 9 - CAPSTONE
* 0 - CAPSTONE BONUS (with No Lidar Data Association)




