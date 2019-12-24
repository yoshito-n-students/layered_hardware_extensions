# layered_hardware_extensions
A ros_control layer implementation for [ros_control_extensions](https://github.com/yoshito-n-students/ros_control_extensions)

## Plugins: layered_hardware_extension_plugins
### layered_hardware_extensions/ExtendedJointLimitsLayer
* implements procedures for joint_limits_interface (i.e. command saturation for hardware_interface::{Position, Velocity, Effort}JointInterface) & [joint_limits_interface_extensions](https://github.com/yoshito-n-students/ros_control_extensions/tree/master/joint_limits_interface_extensions) (i.e. for hardware_interface::PosVelJointInterface & [hardware_interface_extensions](https://github.com/yoshito-n-students/ros_control_extensions/tree/master/hardware_interface_extensions)::PosVelEffJointInterface)
* supports only hard limits. will support soft limits soon.

### layered_hardware_extensions/Dummy{PosVel, PosVelEff}ActuatorLayer
* implements dummy actuators position-controlled with profile velocity (& effort limit)