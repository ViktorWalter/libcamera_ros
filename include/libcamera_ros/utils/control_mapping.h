#pragma once
#include <string>
#include <libcamera/control_ids.h>


libcamera::controls::AeExposureModeEnum get_exposure_mode(const std::string &mode);

