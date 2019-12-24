#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_extensions/extended_joint_limits_layer.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(layered_hardware_extensions::ExtendedJointLimitsLayer,
                       layered_hardware::LayerBase);