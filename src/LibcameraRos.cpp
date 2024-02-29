/* includes //{ */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

/* libcamera //{ */

#include <libcamera/base/shared_fd.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/span.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

//}

#include <utils/clamp.hpp>
#include <utils/format_mapping.hpp>
#include <utils/pretty_print.hpp>
#include <utils/type_extent.hpp>
#include <utils/types.hpp>


//}

namespace libcamera_ros
{

libcamera::StreamRole
get_role(const std::string &role)
{
  static const std::unordered_map<std::string, libcamera::StreamRole> roles_map = {
    {"raw", libcamera::StreamRole::Raw},
    {"still", libcamera::StreamRole::StillCapture},
    {"video", libcamera::StreamRole::VideoRecording},
    {"viewfinder", libcamera::StreamRole::Viewfinder},
  };

  try {
    return roles_map.at(role);
  }
  catch (const std::out_of_range &) {
    throw std::runtime_error("invalid stream role: \"" + role + "\"");
  }
}

/* class LibcameraRos //{ */

class LibcameraRos : : public nodelet::Nodelet
{
public:
  virtual void onInit();
  ~LibcameraRos();

private:
  ros::NodeHandle   nh_;

  libcamera::CameraManager camera_manager;
  std::shared_ptr<libcamera::Camera> camera;
  libcamera::Stream *stream;
  std::shared_ptr<libcamera::FrameBufferAllocator> allocator;
  std::vector<std::unique_ptr<libcamera::Request>> requests;
  std::mutex request_lock;

  struct buffer_info_t
  {
    void *data;
    size_t size;
  };
  std::unordered_map<const libcamera::FrameBuffer *, buffer_info_t> buffer_info;

  // timestamp offset (ns) from camera time to system time
  int64_t time_offset = 0;

  void
  requestComplete(libcamera::Request *request);
};

//}

LibcameraRos::onInit() {
  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();
  
  // start camera manager and check for cameras
  camera_manager.start();
  if (camera_manager.cameras().empty())
    throw std::runtime_error("no cameras available");

  if (!camera)
    throw std::runtime_error("failed to find camera");

  if (camera->acquire())
    throw std::runtime_error("failed to acquire camera");

  // configure camera stream
  std::unique_ptr<libcamera::CameraConfiguration> cfg =
    camera->generateConfiguration({get_role(get_parameter("role").as_string())});

  if (!cfg)
    throw std::runtime_error("failed to generate configuration");

  libcamera::StreamConfiguration &scfg = cfg->at(0);
  // store full list of stream formats
  const libcamera::StreamFormats &stream_formats = scfg.formats();
  const std::vector<libcamera::PixelFormat> &pixel_formats = scfg.formats().pixelformats();
  const std::string format = get_parameter("format").as_string();
  if (format.empty()) {
    ROS_INFO_STREAM(stream_formats);
    // check if the default pixel format is supported
    if (format_type(scfg.pixelFormat) == FormatType::NONE) {
      // find first supported pixel format available by camera
      const auto result = std::find_if(
        pixel_formats.begin(), pixel_formats.end(),
        [](const libcamera::PixelFormat &fmt) { return format_type(fmt) != FormatType::NONE; });

      if (result == pixel_formats.end())
        throw std::runtime_error("camera does not provide any of the supported pixel formats");

      scfg.pixelFormat = *result;
    }

    ROS_WARN_STREAM("no pixel format selected, using default: \"" << scfg.pixelFormat << "\"");
  }
  else {
    // get pixel format from provided string
    const libcamera::PixelFormat format_requested = libcamera::PixelFormat::fromString(format);
    if (!format_requested.isValid()) {
      ROS_INFO_STREAM(stream_formats);
      throw std::runtime_error("invalid pixel format: \"" + format + "\"");
    }
    // check that requested format is supported by camera
    if (std::find(pixel_formats.begin(), pixel_formats.end(), format_requested) ==
        pixel_formats.end()) {
      ROS_INFO_STREAM(stream_formats);
      throw std::runtime_error("pixel format \"" + format + "\" is unsupported by camera");
    }
    // check that requested format is supported by node
    if (format_type(format_requested) == FormatType::NONE)
      throw std::runtime_error("pixel format \"" + format + "\" is unsupported by node");
    scfg.pixelFormat = format_requested;
  }

  const libcamera::Size size(get_parameter("width").as_int(), get_parameter("height").as_int());
  if (size.isNull()) {
    ROS_INFO_STREAM(scfg);
    scfg.size = scfg.formats().sizes(scfg.pixelFormat).back();
    ROS_WARN_STREAM("no dimensions selected, auto-selecting: \"" << scfg.size << "\"");
  }
  else {
    scfg.size = size;
  }

  // store selected stream configuration
  const libcamera::StreamConfiguration selected_scfg = scfg;

  switch (cfg->validate()) {
  case libcamera::CameraConfiguration::Valid:
    break;
  case libcamera::CameraConfiguration::Adjusted:
    if (selected_scfg.pixelFormat != scfg.pixelFormat)
      ROS_INFO_STREAM(stream_formats);
    if (selected_scfg.size != scfg.size)
      ROS_INFO_STREAM(scfg);
    ROS_WARN_STREAM("stream configuration adjusted from \""
                                       << selected_scfg.toString() << "\" to \"" << scfg.toString()
                                       << "\"");
    break;
  case libcamera::CameraConfiguration::Invalid:
    throw std::runtime_error("failed to valid stream configurations");
    break;
  }

  if (camera->configure(cfg.get()) < 0)
    throw std::runtime_error("failed to configure streams");

  ROS_INFO_STREAM("camera \"" << camera->id() << "\" configured with " << scfg.toString() << " stream");

  // format camera name for calibration file
  const libcamera::ControlList &props = camera->properties();
  std::string cname = camera->id() + '_' + scfg.size.toString();
  const std::optional<std::string> model = props.get(libcamera::properties::Model);
  if (model)
    cname = model.value() + '_' + cname;

  // clean camera name of non-alphanumeric characters
  cname.erase(
    std::remove_if(cname.begin(), cname.end(), [](const char &x) { return std::isspace(x); }),
    cname.cend());
  std::replace_if(
    cname.begin(), cname.end(), [](const char &x) { return !std::isalnum(x); }, '_');

  if (!cim.setCameraName(cname))
    throw std::runtime_error("camera name must only contain alphanumeric characters");

  // allocate stream buffers and create one request per buffer
  stream = scfg.stream();

  allocator = std::make_shared<libcamera::FrameBufferAllocator>(camera);
  allocator->allocate(stream);

  for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator->buffers(stream)) {
    std::unique_ptr<libcamera::Request> request = camera->createRequest();
    if (!request)
      throw std::runtime_error("Can't create request");

    // multiple planes of the same buffer use the same file descriptor
    size_t buffer_length = 0;
    int fd = -1;
    for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
      if (plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset)
        throw std::runtime_error("invalid offset");
      buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);
      if (!plane.fd.isValid())
        throw std::runtime_error("file descriptor is not valid");
      if (fd == -1)
        fd = plane.fd.get();
      else if (fd != plane.fd.get())
        throw std::runtime_error("plane file descriptors differ");
    }

    // memory-map the frame buffer planes
    void *data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED)
      throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
    buffer_info[buffer.get()] = {data, buffer_length};

    if (request->addBuffer(stream, buffer.get()) < 0)
      throw std::runtime_error("Can't set buffer for request");

    requests.push_back(std::move(request));
  }

  // register callback
  camera->requestCompleted.connect(this, &LibcameraRos::requestComplete);

  // start camera and queue all requests
  if (camera->start())
    throw std::runtime_error("failed to start camera");

  for (std::unique_ptr<libcamera::Request> &request : requests)
    camera->queueRequest(request.get());

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[LibcameraRos]: initialized");
  ROS_INFO("[LibcameraRos]: --------------------");
}

LibcameraRos::~LibcameraRos()
{
  camera->requestCompleted.disconnect();
  request_lock.lock();
  if (camera->stop())
    std::cerr << "failed to stop camera" << std::endl;
  request_lock.unlock();
  camera->release();
  camera_manager.stop();
  for (const auto &e : buffer_info)
    if (munmap(e.second.data, e.second.size) == -1)
      std::cerr << "munmap failed: " << std::strerror(errno) << std::endl;
}

void
LibcameraRos::requestComplete(libcamera::Request *request)
{
  request_lock.lock();

  if (request->status() == libcamera::Request::RequestComplete) {
    assert(request->buffers().size() == 1);

    // get the stream and buffer from the request
    const libcamera::FrameBuffer *buffer = request->findBuffer(stream);
    const libcamera::FrameMetadata &metadata = buffer->metadata();
    size_t bytesused = 0;
    for (const libcamera::FrameMetadata::Plane &plane : metadata.planes())
      bytesused += plane.bytesused;

    // set time offset once for accurate timing using the device time
    if (time_offset == 0)
      time_offset = this->now().nanoseconds() - metadata.timestamp;

    // send image data
    std_msgs::msg::Header hdr;
    hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
    hdr.frame_id = "camera";
    const libcamera::StreamConfiguration &cfg = stream->configuration();

    auto msg_img = std::make_unique<sensor_msgs::msg::Image>();
    auto msg_img_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();

    if (format_type(cfg.pixelFormat) == FormatType::RAW) {
      // raw uncompressed image
      assert(buffer_info[buffer].size == bytesused);
      msg_img->header = hdr;
      msg_img->width = cfg.size.width;
      msg_img->height = cfg.size.height;
      msg_img->step = cfg.stride;
      msg_img->encoding = get_ros_encoding(cfg.pixelFormat);
      msg_img->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
      msg_img->data.resize(buffer_info[buffer].size);
      memcpy(msg_img->data.data(), buffer_info[buffer].data, buffer_info[buffer].size);

      // compress to jpeg
      if (pub_image_compressed->get_subscription_count())
        cv_bridge::toCvCopy(*msg_img)->toCompressedImageMsg(*msg_img_compressed);
    }
    else if (format_type(cfg.pixelFormat) == FormatType::COMPRESSED) {
      // compressed image
      assert(bytesused < buffer_info[buffer].size);
      msg_img_compressed->header = hdr;
      msg_img_compressed->format = get_ros_encoding(cfg.pixelFormat);
      msg_img_compressed->data.resize(bytesused);
      memcpy(msg_img_compressed->data.data(), buffer_info[buffer].data, bytesused);

      // decompress into raw rgb8 image
      if (pub_image->get_subscription_count())
        cv_bridge::toCvCopy(*msg_img_compressed, "rgb8")->toImageMsg(*msg_img);
    }
    else {
      throw std::runtime_error("unsupported pixel format: " +
                               stream->configuration().pixelFormat.toString());
    }

    pub_image->publish(std::move(msg_img));
    pub_image_compressed->publish(std::move(msg_img_compressed));

    sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
    ci.header = hdr;
    pub_ci->publish(ci);
  }
  else if (request->status() == libcamera::Request::RequestCancelled) {
    ROS_ERROR_STREAM("request '" << request->toString() << "' cancelled");
  }

  // queue the request again for the next frame
  request->reuse(libcamera::Request::ReuseBuffers);

  camera->queueRequest(request);

  request_lock.unlock();
}

} // namespace libcamera_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(libcamera_ros::LibcameraRos, nodelet::Nodelet);
