# ar2_arm_interface

This package is the hardware interface for the AR2 arm.

Thanks to [ANNIN](https://www.anninrobotics.com/home) with its open-source arm AR series, and Dexter Ong with his repository of [MoveIt config of AR3](https://github.com/ongdexter/ar3_core), this package has a solid baseline. I bought a Chinese version of AR3, which takes the hardware controller from AR2 and others from AR3. So I can’t leverage Dexter’s ar3_core to control the real-world arm, and this finally lead me to implement this package. I hope it could be a complement to the AR series, and extend AR2 to the ROS world.

## Overview
### MoveIt config for AR2 arm: ar3_core
I forked the repository “ar3_core” and made some adaptions like joint limits, pose group, and URDF to meet AR2.

### Arduino controller: sketch_ar2_control
With a thorough refactor, it can catch up with the frequency of the hardware interface by removing many logics that are redundant to MoveIt. Meanwhile, it keeps the “MJ” protocol as exact as before, which enables communication to “AR2.py” as well.

### Arm hardware interface: ar2_hardware_interface
A bridge between Arduino controller and MoveIt. It follows the architect of ros_controller, and uses rosserial to set up the link with Arduino Mega2560.

## Prerequisites
Communication between arm hardware interface and Arduino Mega2560 relies on rosserial, please make sure it is installed.

```
sudo apt install ros-<ros_distro>-rosserial
sudo apt install ros-<ros_distro>-rosserial-arduino
```

## Buildup
1.	Clone Arduino sketch and upload to Mega2560

The repository can be found at the link:
https://github.com/xinjuezou-whi/Arduino-sketch_ar2_control

```
git clone https://github.com/xinjuezou-whi/Arduino-sketch_ar2_control.git
```

compile and upload through Arduino IDE

2.	Clone ar3_core

The forked and modified repository can be found at the link:
https://github.com/xinjuezou-whi/ar3_core

```
cd <your_workspace>/src
git clone https://github.com/xinjuezou-whi/ar3_core.git
```

3.	Clone ar2_hardware_interface

```
cd <your_workspace>/src
git clone https://github.com/xinjuezou-whi/ar2_arm_interface.git
```

4.	Build

```
cd <your_workspace>
catkin build
```
or
```
cd <your_workspace>
catkin_make
```
based on your environment setting

## Usage
### View the URDF in RViz
If you want to check the TF relation among joints, viewing the URDF is a good choice.

```
cd <your_workspace>
roslaunch ar3_description urdf.launch
```
![image](https://user-images.githubusercontent.com/72239958/183247213-5720789e-2100-4b05-984a-3cb20e74f99f.png)


### MoveIt demo in RViz
It is recommended to start with this to explore planning with MoveIt in RViz, if MoveIt is new to you. It will bring up a simulated arm in RViz for exploring the motion planning of MoveIt.

```
cd <your_workspace>
roslaunch ar3_moveit_config demo.launch
```
![image](https://user-images.githubusercontent.com/72239958/183247226-71a393b1-c512-4773-b8e3-832acdab59ad.png)


### Control the real-world arm in RViz
1.	Start the ar2_hardware_interface, which will load configs and the robot description, meanwhile, also initialize communication with the Arduino Mega2560

Start with find home first and close-loop mode:

```
cd <your_workspace>
roslaunch ar2_arm_interface ar2_arm_hardware_interface.launch home:=true close_loop:=true
```

Since AR2 takes step motors as its actuator, the home position cannot be memorized after power off. Therefore, it is recommended to always find the home at every startup.

2.	Start the MoveIt
```
cd <your_workspace>
roslaunch ar3_moveit_config ar3_moveit_bringup.launch
```
![ar2](https://user-images.githubusercontent.com/72239958/183247871-0b461c60-7794-4c48-bb19-d14dfafafe43.gif)

While the MoveIt is running, the real-world arm can be controlled through Plan panel in RViz.
