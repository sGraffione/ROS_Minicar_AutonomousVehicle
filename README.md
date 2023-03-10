# ROS_Minicar_AutonomousVehicle
Implementation of Autonomous system with ROS on a Minicar provided of a Raspberry Pi 3.
## Dependencies
- Ubuntu (or similar) 18.04
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [WiringPi](http://wiringpi.com/wiringpi-updated-for-the-pi-v3plus/) for Raspberry Pi 3
- Eigen3 (included in ROS)
- [Casadi](https://github.com/casadi/casadi)


 **Install IPOPT to solve non-linear problems**

```
sudo apt-get install coinor-libipopt-dev
```
**Install casadi by compiling from source**

```
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
sudo apt-get install swig ipython python-dev python-numpy python-scipy python-matplotlib

git clone https://github.com/casadi/casadi.git -b master casadi

cd casadi
mkdir build
cd build

cmake -DWITH_PYTHON=ON -DWITH_IPOPT=true ..
```
Now build from source and install
```
make
sudo make install
```

Full installantion guide at https://github.com/casadi/casadi/wiki/InstallationLinux


**Download cmake files from [use_ext_libraries](/use_ext_libraries) directory**

Open a terminal

```
cd /opt/ros/melodic/share/cmake_modules/cmake/Modules/
sudo cp ~/Download/FindEigen.cmake .
sudo cp ~/Download/FindCASADI.cmake .
```
