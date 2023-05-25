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

git clone https://github.com/casadi/casadi.git -b main casadi

cd casadi
mkdir build
cd build

cmake -DWITH_PYTHON=ON -DWITH_IPOPT=true ..
```
*To compile Casadi is required cmake version > 3.15. To update your cmake version to the lateste one, follow [this guide](https://apt.kitware.com/).*

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

# Common issues
**Missing bcm2835.h**

Install the missing library with the following commands

```
cd ~                  
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.58.tar.gz                       
tar xvfz bcm2835-1.58.tar.gz;                      
cd bcm2835-1.58;                       
./configure;                      
make;        
sudo make install
```

