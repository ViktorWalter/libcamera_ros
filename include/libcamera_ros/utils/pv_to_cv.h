#pragma once
#include <libcamera/controls.h>


namespace XmlRpc
{
class XmlRpcValue;
}

template<typename T>
libcamera::ControlValue pv_to_cv(const T &parameter, const libcamera::ControlType &type);
