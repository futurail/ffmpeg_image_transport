#ifndef CONVERTERS_H_L7OWNAZ8
#define CONVERTERS_H_L7OWNAZ8

#include <memory>
#include <rclcpp/rclcpp.hpp>

// #include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <numeric>
#include <sstream>
#include <string>
#include <type_traits>
#include <typeindex>
#include <utility>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace py = pybind11;

static inline bool is_ros_msg_type(pybind11::handle src,
                                   const std::string &msg_type_name) {
  
  if (!py::hasattr(src, "_type")) {
    return false;
  }
  std::string msg_type(src.attr("_type").cast<std::string>());
  
  if (msg_type != msg_type_name) {
    return false;
  }
  return true;
}
namespace pybind11 {
namespace detail {
template <>
struct type_caster<std_msgs::msg::Header> {
 public:
  PYBIND11_TYPE_CASTER(std_msgs::msg::Header, _("std_msgs::msg::Header"));

  // python -> cpp
  bool load(handle src, bool) {
    // if (!is_ros_msg_type(src, "std_msgs/msg/Header")) {
    //   return false;
    // }
    
    value.stamp = src.attr("stamp").cast<rclcpp::Time>();
    value.frame_id = src.attr("frame_id").cast<std::string>();
    return true;
  }

  // cpp -> python
  static handle cast(std_msgs::msg::Header header,
                     return_value_policy policy,
                     handle parent) {
    object mod = module::import("std_msgs.msg._Header");
    object MsgType = mod.attr("Header");
    object msg = MsgType();
    
    msg.attr("stamp") = pybind11::cast(header.stamp);
    // avoid !!python/unicode problem.
    // msg.attr("frame_id") = pybind11::cast(header.frame_id);
    msg.attr("frame_id") =
        pybind11::bytes(reinterpret_cast<const char *>(&header.frame_id[0]),
                        header.frame_id.size());
    msg.inc_ref();
    return msg;
  }
};
template <>
struct type_caster<rclcpp::Time> {
 public:
  PYBIND11_TYPE_CASTER(rclcpp::Time, _("rclcpp::Time"));

  // python -> cpp
  bool load(handle src, bool) {
    
    PyObject *obj = src.ptr();
    if (!PyObject_HasAttrString(obj, "sec")) {
      return false;
    }
   
    if (!PyObject_HasAttrString(obj, "nanosec")) {
      return false;
    }

    value = rclcpp::Time{
        (src.attr("sec")).cast<int64_t>(),
        (src.attr("nanosec")).cast<uint32_t>()
    };
   
    return true;
  }

  // cpp -> python
  static handle cast(const rclcpp::Time &src, return_value_policy, handle) {
    py::object pyts = py::module_::import("rclpy").attr("Time")(
        src.seconds(), src.nanoseconds()
    );
    pyts.inc_ref();  // Increment the reference count to manage the object's lifetime
    return pyts;
  }
};
template <>
struct type_caster<sensor_msgs::msg::Image> {
 public:
  PYBIND11_TYPE_CASTER(sensor_msgs::msg::Image, _("sensor_msgs::msg::Image"));

  bool load(handle src, bool) {
    // if (!is_ros_msg_type(src, "sensor_msgs/msg/Image")) {
    //   return false;
    // }
    value.header = src.attr("header").cast<std_msgs::msg::Header>();
    value.height = src.attr("height").cast<uint32_t>();
    value.width = src.attr("width").cast<uint32_t>();
    value.encoding = src.attr("encoding").cast<std::string>();
    value.is_bigendian = src.attr("is_bigendian").cast<int>();
    value.step = src.attr("step").cast<uint32_t>();
     
    // std::string data_str = src.attr("data").cast<std::string>(); 
    
    py::array_t<uint8_t> data_array = src.attr("data").cast<py::array_t<uint8_t>>();
        auto ptr = static_cast<uint8_t*>(data_array.request().ptr);
    size_t size = data_array.size();
    
   
    // Insert data into value.data vector
    value.data.insert(value.data.end(), ptr, ptr + size);
    // value.data.insert(value.data.end(),
    //                   data_str.c_str(),
    //                   data_str.c_str() + data_str.length());
    
    return true;
  }

  static handle cast(sensor_msgs::msg::Image cpp_msg,
                     return_value_policy policy,
                     handle parent) {
    object mod = module::import("sensor_msgs.msg._Image");
    object MsgType = mod.attr("Image");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("height") = pybind11::cast(cpp_msg.height);
    msg.attr("width") = pybind11::cast(cpp_msg.width);
    msg.attr("encoding") = pybind11::bytes(cpp_msg.encoding);
    msg.attr("is_bigendian") = pybind11::cast(cpp_msg.is_bigendian);
    msg.attr("step") = pybind11::cast(cpp_msg.step);
    msg.attr("data") = pybind11::bytes(
        reinterpret_cast<const char *>(&cpp_msg.data[0]), cpp_msg.data.size());
    msg.inc_ref();
    return msg;
  }
};

}  // namespace detail
}  // namespace pybind11

#endif /* end of include guard: CONVERTERS_H_L7OWNAZ8 */
