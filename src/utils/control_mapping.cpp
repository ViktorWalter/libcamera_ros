#include <libcamera_ros/utils/control_mapping.h>
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <ros/ros.h>

libcamera::controls::AeExposureModeEnum get_exposure_mode(const std::string &mode){
  static const std::unordered_map<std::string, libcamera::controls::AeExposureModeEnum> mode_map = {
    {"normal", libcamera::controls::AeExposureModeEnum::ExposureNormal},
    {"short", libcamera::controls::AeExposureModeEnum::ExposureShort},
    {"long", libcamera::controls::AeExposureModeEnum::ExposureLong},
    {"custom", libcamera::controls::AeExposureModeEnum::ExposureCustom},
  };

  try {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range &) {
    ROS_ERROR_STREAM("invalid exposure mode: \"" << mode << "\"");
    throw std::runtime_error("invalid exposure mode: \"" + mode + "\"");
  }
}

