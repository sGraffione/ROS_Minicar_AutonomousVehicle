# ROS_Minicar_AutonomousVehicle
Implementation of Autonomous system with ROS on a Minicar provided of a Raspberry Pi 3.
## Dependencies
- [WiringPi](http://wiringpi.com/wiringpi-updated-for-the-pi-v3plus/) for Raspberry Pi 3
- [Adafruit_PCA9685](https://github.com/adafruit/Adafruit_CircuitPython_PCA9685)
- [Adafruit_servokit](https://docs.circuitpython.org/projects/servokit/en/latest/index.html)
- Eigen3 (included in ROS)
- [Casadi](https://github.com/casadi/casadi)


**Install casadi by compiling from source**

```
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
sudo apt-get install swig ipython python-dev python-numpy python-scipy python-matplotlib

git clone https://github.com/casadi/casadi.git -b master casadi

cd casadi
mkdir build
cd build

cmake -DWITH_PYTHON=ON ..
```

Full installantion guide at https://github.com/casadi/casadi/wiki/InstallationLinux


**Download cmake files from [use_ext_libraries](/use_ext_libraries) directory**

Open a terminal

```
cd /opt/ros/melodic/share/cmake_modules/cmake/Modules/
sudo cp ~/Download/FindEigen.cmake .
sudo cp ~/Download/FindCASADI.cmake .
```
