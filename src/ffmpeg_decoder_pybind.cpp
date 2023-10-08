#include <pybind11/pybind11.h>

#include "ffmpeg_image_transport/ffmpeg_decoder.hpp"

namespace py = pybind11;
namespace ffmpeg_image_transport {

PYBIND11_MODULE(ffmpeg_bindings, m) {
    py::class_<FFMPEGDecoder>(m, "FFMPEGDecoder")
        .def(py::init<>()); 
      
}

} // namespace ffmpeg_image_transport