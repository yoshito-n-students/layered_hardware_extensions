#ifndef LAYERED_HARDWARE_EXTENSIONS_EXTENDED_DUMMY_ACTUATOR_LAYERS_HPP
#define LAYERED_HARDWARE_EXTENSIONS_EXTENDED_DUMMY_ACTUATOR_LAYERS_HPP

#include <cmath>

#include <layered_hardware/dummy_actuator_layers.hpp>
#include <layered_hardware_extensions/common_namespaces.hpp>
#include <ros/duration.h>

namespace layered_hardware_extensions {

// command writers

struct ActuatorPosVelCommandWriter {
  static void write(lh::DummyActuatorData *const data, const ros::Duration &period) {
    if (!std::isnan(data->pos_cmd) && !std::isnan(data->vel_cmd)) {
      const double vel_limit((data->pos_cmd - data->pos) / period.toSec());
      data->vel = (vel_limit > 0.) ? std::min(vel_limit, std::abs(data->vel_cmd))
                                   : std::max(vel_limit, -std::abs(data->vel_cmd));
      data->pos += data->vel * period.toSec();
      data->eff = 0.;
    }
  }
};

struct ActuatorPosVelEffCommandWriter {
  static void write(lh::DummyActuatorData *const data, const ros::Duration &period) {
    if (!std::isnan(data->pos_cmd) && !std::isnan(data->vel_cmd) && !std::isnan(data->eff_cmd)) {
      const double vel_limit((data->pos_cmd - data->pos) / period.toSec());
      data->vel = (vel_limit > 0.) ? std::min(vel_limit, std::abs(data->vel_cmd))
                                   : std::max(vel_limit, -std::abs(data->vel_cmd));
      data->pos += data->vel * period.toSec();
      data->eff = (data->vel > 0.) ? std::abs(data->eff_cmd) : -std::abs(data->eff_cmd);
    }
  }
};

// layer definitions
typedef lh::DummyActuatorLayer< ActuatorPosVelCommandWriter > DummyPosVelActuatorLayer;
typedef lh::DummyActuatorLayer< ActuatorPosVelEffCommandWriter > DummyPosVelEffActuatorLayer;
} // namespace layered_hardware_extensions

#endif