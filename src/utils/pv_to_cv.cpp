#include <libcamera_ros/utils/pv_to_cv.h>
#include <libcamera_ros/utils/types.h>
#include <cstdint>
#include <libcamera/base/span.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <ros/ros.h>
#include <vector>


libcamera::ControlValue pv_to_cv_int_array(const std::vector<int64_t> &values, const libcamera::ControlType &type) {
  // convert to Span (Integer32, Integer64) or geometric type Rectangle, Size
  switch (type) {
    case libcamera::ControlTypeInteger32:
      return {
        libcamera::Span<const CTInteger32>(std::vector<CTInteger32>(values.begin(), values.end()))};
    case libcamera::ControlTypeInteger64:
      return {libcamera::Span<const CTInteger64>(values)};
    case libcamera::ControlTypeRectangle:
      return {libcamera::Rectangle(values[0], values[1], values[2], values[3])};
    case libcamera::ControlTypeSize:
      return {libcamera::Size(values[0], values[1])};
    default:
      return {};
  }
}

template<typename T>
libcamera::ControlValue pv_to_cv(const T &parameter, const libcamera::ControlType &type){
  if (std::is_same<T, bool>::value){
    return {parameter};
  }else if(std::is_same<T, int>::value){
    if (type == libcamera::ControlTypeInteger32)
      return {CTInteger32(parameter)};
    else if (type == libcamera::ControlTypeInteger64)
      return {CTInteger64(parameter)};
    else
      return {};
  }else if(std::is_same<T, double>::value){
    return {CTFloat(parameter)};
  }else if(std::is_same<T, std::string>::value){
    return {parameter};
  }else if(std::is_same<T, std::vector<int>>::value){
    return pv_to_cv_int_array(std::vector<int64_t>(
      parameter.begin(), parameter.end()), type);
  }else if(std::is_same<T, std::vector<double>>::value){
    // convert to float vector
    return {libcamera::Span<const CTFloat>(std::vector<CTFloat>(
          parameter.begin(), parameter.end()))};
  }else if(std::is_same<T, std::vector<std::string>>::value){
    return {libcamera::Span<const CTString>(parameter)};
  }else {
    return {};
  }
}
