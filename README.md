![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/humble.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/jazzy.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/rolling.yml/badge.svg)


This branch implements command the joint configuration to the [lbr_stack](https://github.com/idra-lab/lbr_fri_ros2_stack) KUKA hardware interface so it allows `Cartesian Impedance Control` as well as `Joint Position Control` depending on the command input chosen as `client_command_mode` in the hardware interface.

Check out their use in the KUKA LBR example [here](https://github.com/idra-lab/kuka_impedance)!  

The structure of the code and some libraries have been taken from the repo [Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).
