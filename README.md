# ROS_Minicar_AutonomousVehicle
Implementation of Autonomous system with ROS on a Minicar provided of a Raspberry Pi 3.
## Dependencies
- [WiringPi](http://wiringpi.com/wiringpi-updated-for-the-pi-v3plus/) for Raspberry Pi 3
- [Adafruit_PCA9685](https://github.com/adafruit/Adafruit_CircuitPython_PCA9685)
- [Adafruit_servokit](https://docs.circuitpython.org/projects/servokit/en/latest/index.html)
- Eigen3 (included in ROS)
- [Casadi](https://github.com/casadi/casadi)

**Download cmake files from [use_ext_libraries](/use_ext_libraries) directory**

Open a terminal

```
cd /opt/ros/melodic/share/cmake_modules/cmake/Modules/
sudo cp ~/Download/FindEigen.cmake .
sudo cp ~/Download/FindCASADI.cmake .
```
