/* includes //{ */
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

#include <libcamera_ros/utils/clamp.hpp>
#include <libcamera_ros/utils/format_mapping.hpp>
#include <libcamera_ros/utils/pretty_print.hpp>
#include <libcamera_ros/utils/type_extent.hpp>
#include <libcamera_ros/utils/types.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
//}

namespace libcamera_ros
{

  /* get_role() method //{ */
  libcamera::StreamRole get_role(const std::string &role)
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

  //}

  /* getParamCheck() method //{ */
  template <typename T>
  bool getParamCheck(const ros::NodeHandle& nh, const std::string& node_name, const std::string& param_name, T& param_out)
  {
    const bool res = nh.getParam(param_name, param_out);
    if (!res)
      ROS_ERROR_STREAM("[" << node_name << "]: Could not load compulsory parameter '" << param_name << "'");
    else
      ROS_INFO_STREAM("[" << node_name << "]: Loaded parameter '" << param_name << "': " << param_out);
    return res;
  }

  template <typename T>
  bool getParamCheck(const ros::NodeHandle& nh, const std::string& node_name, const std::string& param_name, T& param_out, const T& param_default)
  {
    const bool res = nh.getParam(param_name, param_out);
    if (!res)
      param_out = param_default;
    ROS_INFO_STREAM("[" << node_name << "]: Loaded parameter '" << param_name << "': " << param_out);
    return res;
  }

  //}

  /* class LibcameraRos //{ */

  class LibcameraRos : public nodelet::Nodelet{
    public:
      virtual void onInit();
      ~LibcameraRos();

    private:
      ros::NodeHandle   nh_;

      libcamera::CameraManager camera_manager_;
      std::shared_ptr<libcamera::Camera> camera_;
      libcamera::Stream *stream_;
      std::shared_ptr<libcamera::FrameBufferAllocator> allocator_;
      std::vector<std::unique_ptr<libcamera::Request>> requests_;
      std::mutex request_lock_;

      std::string frame_id_;

      struct buffer_info_t
      {
        void *data;
        size_t size;
      };
      std::unordered_map<const libcamera::FrameBuffer *, buffer_info_t> buffer_info_;

      std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

      image_transport::CameraPublisher image_pub_;
			std::mutex image_pub_mutex_;

      void requestComplete(libcamera::Request *request);
  };

  //}

  /* LibcameraRos::onInit method //{ */

  void LibcameraRos::onInit() {
    /* obtain node handle */
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();
     
    /* load parameters //{ */

    bool success = true;
    std::string camera_name;
    std::string camera_role;
    std::string camera_format;
    std::string calib_url;
		int width;
    int height;

    success = success && getParamCheck(nh_, "LibcameraRos", "camera_name", camera_name);
    success = success && getParamCheck(nh_, "LibcameraRos", "camera_role", camera_role);
    success = success && getParamCheck(nh_, "LibcameraRos", "camera_format", camera_format);
    success = success && getParamCheck(nh_, "LibcameraRos", "frame_id", frame_id_);
    success = success && getParamCheck(nh_, "LibcameraRos", "calib_url", calib_url);
    success = success && getParamCheck(nh_, "LibcameraRos", "width", width);
    success = success && getParamCheck(nh_, "LibcameraRos", "height", height);

    if (!success)
    {
      ROS_ERROR("[LibcameraRos]: Some compulsory parameters were not loaded successfully, ending the node");
      ros::shutdown();
      return;
    }

    //}

    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh_, camera_name, calib_url);

    /* initialize publishers //{ */

    image_transport::ImageTransport it(nh_);
    image_pub_ = it.advertiseCamera("image_raw", 5);

    //}

    // start camera manager and check for cameras
    camera_manager_.start();
    if (camera_manager_.cameras().empty())
      throw std::runtime_error("no cameras available");

    camera_ = camera_manager_.get(camera_name);
    if (!camera_) {
      ROS_INFO_STREAM(camera_manager_);
      throw std::runtime_error("camera with name " + camera_name + " does not exist");
    }

    if (camera_->acquire())
      throw std::runtime_error("failed to acquire camera");

    // configure camera stream
    std::unique_ptr<libcamera::CameraConfiguration> cfg =
      camera_->generateConfiguration({get_role(camera_role)});

    if (!cfg)
      throw std::runtime_error("failed to generate configuration");

    libcamera::StreamConfiguration &scfg = cfg->at(0);
    // store full list of stream formats
    const libcamera::StreamFormats &stream_formats = scfg.formats();
    const std::vector<libcamera::PixelFormat> &pixel_formats = scfg.formats().pixelformats();
    const std::string format = camera_format;
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

    const libcamera::Size size(width, height);
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

    if (camera_->configure(cfg.get()) < 0)
      throw std::runtime_error("failed to configure streams");

    ROS_INFO_STREAM("camera \"" << camera_->id() << "\" configured with " << scfg.toString() << " stream");

    // allocate stream buffers and create one request per buffer
    stream_ = scfg.stream();

    allocator_ = std::make_shared<libcamera::FrameBufferAllocator>(camera_);
    allocator_->allocate(stream_);

    for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator_->buffers(stream_)) {
      std::unique_ptr<libcamera::Request> request = camera_->createRequest();
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
      buffer_info_[buffer.get()] = {data, buffer_length};

      if (request->addBuffer(stream_, buffer.get()) < 0)
        throw std::runtime_error("Can't set buffer for request");

      requests_.push_back(std::move(request));
    }

    // register callback
    camera_->requestCompleted.connect(this, &LibcameraRos::requestComplete);

    // start camera and queue all requests
    if (camera_->start())
      throw std::runtime_error("failed to start camera");

    for (std::unique_ptr<libcamera::Request> &request : requests_)
      camera_->queueRequest(request.get());

    // | --------------------- finish the init -------------------- |

    ROS_INFO("[LibcameraRos]: initialized");
  }
  //}

  /* LibcameraRos::~LibcameraRos() //{ */

  LibcameraRos::~LibcameraRos()
  {
    camera_->requestCompleted.disconnect();
    request_lock_.lock();
    if (camera_->stop())
      std::cerr << "failed to stop camera" << std::endl;
    request_lock_.unlock();
    camera_->release();
    camera_manager_.stop();
    for (const auto &e : buffer_info_)
      if (munmap(e.second.data, e.second.size) == -1)
        std::cerr << "munmap failed: " << std::strerror(errno) << std::endl;
  }
  //}

  /* LibcameraRos::requestComplete() //{ */

	void LibcameraRos::requestComplete(libcamera::Request *request) {
		request_lock_.lock();

		if (request->status() == libcamera::Request::RequestComplete) {
			assert(request->buffers().size() == 1);

			// get the stream and buffer from the request
			const libcamera::FrameBuffer *buffer = request->findBuffer(stream_);
			const libcamera::FrameMetadata &metadata = buffer->metadata();
			size_t bytesused = 0;
			for (const libcamera::FrameMetadata::Plane &plane : metadata.planes())
				bytesused += plane.bytesused;

			// send image data
			std_msgs::Header hdr;
			hdr.stamp = ros::Time().fromNSec(metadata.timestamp);
			hdr.frame_id = frame_id_;
			const libcamera::StreamConfiguration &cfg = stream_->configuration();

			sensor_msgs::Image image_msg; 

			if (format_type(cfg.pixelFormat) == FormatType::RAW) {
				// raw uncompressed image
				assert(buffer_info_[buffer].size == bytesused);
				image_msg.header = hdr;
				image_msg.width = cfg.size.width;
				image_msg.height = cfg.size.height;
				image_msg.step = cfg.stride;
				image_msg.encoding = get_ros_encoding(cfg.pixelFormat);
				image_msg.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
				image_msg.data.resize(buffer_info_[buffer].size);
				memcpy(image_msg.data.data(), buffer_info_[buffer].data, buffer_info_[buffer].size);

			}else {
				throw std::runtime_error("unsupported pixel format: " +
						stream_->configuration().pixelFormat.toString());
			}

			sensor_msgs::CameraInfo cinfo_msg = cinfo_->getCameraInfo();
			cinfo_msg.header = hdr;
			std::scoped_lock lck(image_pub_mutex_);
			image_pub_.publish(image_msg, cinfo_msg);
		}
		else if (request->status() == libcamera::Request::RequestCancelled) {
			ROS_ERROR_STREAM("request '" << request->toString() << "' cancelled");
		}

		// queue the request again for the next frame
		request->reuse(libcamera::Request::ReuseBuffers);
		camera_->queueRequest(request);
		request_lock_.unlock();
	}
  //}

} // namespace libcamera_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(libcamera_ros::LibcameraRos, nodelet::Nodelet);
