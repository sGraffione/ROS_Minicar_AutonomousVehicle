# ROS_Minicar_AutonomousVehicle
Implementation of Autonomous system with ROS on a Minicar provided of a Raspberry Pi 3.
## Dependencies
- Ubuntu (or similar) 18.04
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [WiringPi](https://github.com/WiringPi/WiringPi) for Raspberry Pi 3
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

# Run commands

**It is necessary to run the system as root user.**
First, use 
```
sudo -s
```
to set the command window as root user. Then run,
```
roslaunch package_name minicar.launch
```
to run each modules and make it follows a path defined in [control.cpp](./src/control.cpp).

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

**Missing i2c/*.h**

If you have any library of i2c missing, it means that you have to check if you have installed i2c-tools and if i2c interface is enabled.

First, install these packages
```
sudo apt install -y i2c-tools python3-smbus
sudo apt install libi2c-dev
```
Then, run the command
```
sudo raspi-config
```
Go to ```5 Interfacing options```, select ```I2C``` and choose ```Enable```

Finally, reboot your system to apply changes.

**Missing MPU6050.h**

To correct this error, go to the include folder and run the commands
```
make all
sudo make install
```

**Undefined reference to ...**

Try to recompile the package with the additional

```
catkin_make --force-cmake
```

**WiFi module missing**
If the WiFi module is not detected, first check with
```
dmesg | grep mmc1
```

It can happen to have missing files. If the command returns a missing firmware like the ```brcm/brcmfmac43430-sdio.txt``` you need to copy the file from [here](https://github.com/armbian/firmware/blob/master/brcm/brcmfmac43430-sdio.txt) and create a new file in ```brcm``` folder:

```
sudo nano /lib/firmware/brcm/brcmfmac43430-sdio.txt
```
and paste the content from the previous link.
