![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/humble.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/jazzy.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/rolling.yml/badge.svg)


This repository aim to create a robot independent torque controller based on ROS2 controllers. 
There is a base controller that communicate with the hardware interface of the robot, on top of that controller different types of controllers can be implemented. 
For now only a cartesian impedance controller is implemented but also other type of controllers like for example an admittance controller can be easily added. 

**If you want to use just the KUKA's proprietary controllers sending just the target joint position checkout the `kuka-prop-ctrl` branch.**

Check out their use in the KUKA LBR example [here](https://github.com/idra-lab/kuka_impedance)!  

The structure of the code and some libraries have been taken from the repo [Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).
