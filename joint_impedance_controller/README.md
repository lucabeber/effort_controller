# Joint Impedance Controller

This controller implements a joint impedance controller which take a desired pose and a desired force in the cartesian frame. It uses inverse kinematics to calculate the desired joint positions and then it calculates the desired torques to apply to the joints.

## Example Configuration
Below is an example `controller_manager.yaml` for a controller specific configuration.
```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    joint_impedance_controller:
      type: joint_impedance_controller/JointImpedanceController

    # More controller instances here
    # ...

joint_impedance_controller:
  ros__parameters:
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    ft_sensor_ref_link: "sensor_link"
    compliance_ref_link: "compliance_link"
    joints:
      - joint_01
      - joint_02
      - joint_03
      - joint_04
      - joint_05
      - joint_06
      - joint_07
      
    command_interfaces:
      - effort
    
    state_interfaces:
      - position
      - velocity

    stiffness:
      joint1: 100
      joint2: 100
      joint3: 100
      joint4: 50
      joint5: 50
      joint6: 50

    nullspace_stiffness: 0.0
    compensate_gravity: false
    compensate_coriolis: false


# More controller specifications here
# ...

```

