#ifndef LAYERED_HARDWARE_EXTENSIONS_EXTENDED_JOINT_LIMITS_LAYER_HPP
#define LAYERED_HARDWARE_EXTENSIONS_EXTENDED_JOINT_LIMITS_LAYER_HPP

#include <list>
#include <string>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface_extensions/posveleff_command_interface.hpp>
#include <joint_limits_interface_extensions/joint_limits_interface_extensions.hpp>
#include <layered_hardware/joint_limits_layer.hpp>
#include <layered_hardware_extensions/common_namespaces.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <urdf/model.h>

#include <boost/foreach.hpp>

namespace layered_hardware_extensions {

class ExtendedJointLimitsLayer : public lh::JointLimitsLayer {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    // init the base class
    if (!lh::JointLimitsLayer::init(hw, param_nh, urdf_str)) {
      return false;
    }

    // we do NOT register joint limit interfaces to the hardware
    // to prevent other layers from updating the interfaces
    // because the interfaces are stateful
    /*
    hw->registerInterface(&posvel_iface_);
    ...
    */

    // extract the robot model from the given URDF, which contains joint limits info
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_str)) {
      ROS_ERROR("ExtendedJointLimitsLayer::init(): Failed to parse URDF");
      return false;
    }

    // associate joints already registered in the joint interface of the hardware
    // and joint limits loaded from the URDF.
    // associated pairs will be stored in the limits interface.
    tieJointsAndLimits< hi::PosVelJointInterface, jlie::PosVelJointSaturationHandle,
                        jlie::PosVelJointSoftLimitsHandle >(hw, urdf_model, &posvel_iface_,
                                                            &posvel_soft_iface_);
    tieJointsAndLimits< hie::PosVelEffJointInterface, jlie::PosVelEffJointSaturationHandle,
                        jlie::PosVelEffJointSoftLimitsHandle >(hw, urdf_model, &posveleff_iface_,
                                                               &posveleff_soft_iface_);

    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {
    return lh::JointLimitsLayer::prepareSwitch(start_list, stop_list);
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    lh::JointLimitsLayer::doSwitch(start_list, stop_list);

    // reset position-based joint limits
    // because new position-based controllers may be starting
    posvel_iface_.reset();
    posveleff_iface_.reset();
    posvel_soft_iface_.reset();
    posveleff_soft_iface_.reset();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    lh::JointLimitsLayer::read(time, period);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    lh::JointLimitsLayer::write(time, period);

    // saturate joint commands
    posvel_iface_.enforceLimits(period);
    posveleff_iface_.enforceLimits(period);
    posvel_soft_iface_.enforceLimits(period);
    posveleff_soft_iface_.enforceLimits(period);
  }

private:
  jlie::PosVelJointSaturationInterface posvel_iface_;
  jlie::PosVelEffJointSaturationInterface posveleff_iface_;

  jlie::PosVelJointSoftLimitsInterface posvel_soft_iface_;
  jlie::PosVelEffJointSoftLimitsInterface posveleff_soft_iface_;
};
} // namespace layered_hardware_extensions

#endif