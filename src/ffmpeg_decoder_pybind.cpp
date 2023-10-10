#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>

#include "ffmpeg_image_transport/ffmpeg_decoder.hpp"
#include "ffmpeg_image_transport/ffmpeg_encoder.hpp"
#include "ffmpeg_image_transport/serialization.hpp"
namespace ffmpeg_image_transport
{

namespace py = pybind11;

sensor_msgs::msg::Image cpp2py_img(int width, int height)
{
  sensor_msgs::msg::Image msg;
  msg.width = width;
  msg.height = height;
  msg.encoding = "rgb8";
  msg.step = width;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      msg.data.push_back(width * i + j);
    }
  }
  return msg;
}

ffmpeg_image_transport_msgs::msg::FFMPEGPacket cpp2py_ffmpeg(int width, int height)
{
  ffmpeg_image_transport_msgs::msg::FFMPEGPacket msg;
  msg.header.stamp = rclcpp::Clock().now();  // Set the timestamp to the current time
  msg.header.frame_id = "ffmpeg_frame";      // Set the frame ID
  const std::string  encoding = "h264";
  uint64_t pts = 0;
  uint8_t flags = 0;
  msg.width = width;
  msg.height = height;
  msg.encoding = encoding;
  msg.pts = pts;
  msg.flags = flags;

  // Dummy data (filling with random values for demonstration)
  msg.data.resize(width * height);
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      msg.data.push_back(width * i + j);
    }
  }

  return msg;
}
PYBIND11_MODULE(ffmpeg_bindings, m)
{
  m.doc() = "Combined bindings module";

  // Expose make_img function from ffmpeg_image_transport namespace
  m.def("cpp2py_img", &cpp2py_img);  
  m.def(
    "cpp2py_ffmpeg", &cpp2py_ffmpeg); 
  py::class_<FFMPEGDecoder>(m, "FFMPEGDecoder")
    .def(py::init<>())
    .def("isInitialized", &FFMPEGDecoder::isInitialized)
    .def("initialize", &FFMPEGDecoder::initialize)
    .def("reset", &FFMPEGDecoder::reset)
    .def("decodePacket", &FFMPEGDecoder::decodePacket)
    .def("setMeasurePerformance", &FFMPEGDecoder::setMeasurePerformance)
    .def("printTimers", &FFMPEGDecoder::printTimers)
    .def("resetTimers", &FFMPEGDecoder::resetTimers)
    .def("setLogger", &FFMPEGDecoder::setLogger);

  py::class_<FFMPEGEncoder>(m, "FFMPEGEncoder")
    .def(py::init<>())
    .def("setCodec", &FFMPEGEncoder::setCodec)
    .def("setProfile", &FFMPEGEncoder::setProfile)
    .def("setPreset", &FFMPEGEncoder::setPreset)
    .def("setTune", &FFMPEGEncoder::setTune)
    .def("setQMax", &FFMPEGEncoder::setQMax)
    .def("setBitRate", &FFMPEGEncoder::setBitRate)
    .def("getGOPSize", &FFMPEGEncoder::getGOPSize)
    .def("setGOPSize", &FFMPEGEncoder::setGOPSize)
    .def("setFrameRate", &FFMPEGEncoder::setFrameRate)
    .def("setMeasurePerformance", &FFMPEGEncoder::setMeasurePerformance)
    .def("isInitialized", &FFMPEGEncoder::isInitialized)
    .def("initialize", &FFMPEGEncoder::initialize)
    .def("setLogger", &FFMPEGEncoder::setLogger)

    // .def("setParameters", (void (FFMPEGEncoder::*)(
    //     const std::string&, const std::string&, const std::string&, const std::string&,
    //     int, int64_t, int64_t, const std::string&
    // )) &FFMPEGEncoder::setParameters, "Set parameters with default values")

    .def(
      "setParameters",
      py::overload_cast<
        const std::string &, const std::string &, const std::string &, const std::string &, int,
        int64_t, int64_t, const std::string &>(&FFMPEGEncoder::setParameters))
    .def("encodeImage", py::overload_cast<const Image &>(&FFMPEGEncoder::encodeImage))
    .def("reset", &FFMPEGEncoder::reset)
    .def("printTimers", &FFMPEGEncoder::printTimers)
    .def("resetTimers", &FFMPEGEncoder::resetTimers);
}

}  // namespace ffmpeg_image_transport
