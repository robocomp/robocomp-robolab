// pybind11 binding: exposes PoseDDSPublisher to imu_fusion_dds's Python worker so it
// can publish the fused pose on the zero-copy FastDDS media plane (rc::media::PosePublisher)
// without imu_fusion_dds itself linking FastDDS directly — the EKF/DSR/Phidget code stays
// pure Python (unchanged from imu_fusion), only this thin publish leg is native.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dds_publisher.h"

namespace py = pybind11;

PYBIND11_MODULE(pose_dds_native, m)
{
    m.doc() = "Zero-copy FastDDS publisher for imu_fusion_dds's fused pose (PoseFrame)";

    py::class_<PoseDDSPublisher::Config>(m, "Config")
        .def(py::init<>())
        .def_readwrite("domain_id", &PoseDDSPublisher::Config::domain_id)
        .def_readwrite("topic", &PoseDDSPublisher::Config::topic)
        .def_readwrite("history_depth", &PoseDDSPublisher::Config::history_depth)
        .def_readwrite("shared_memory_only", &PoseDDSPublisher::Config::shared_memory_only)
        .def_readwrite("data_sharing", &PoseDDSPublisher::Config::data_sharing);

    py::class_<PoseDDSPublisher>(m, "PoseDDSPublisher")
        .def(py::init<>())
        .def("init", &PoseDDSPublisher::init, py::arg("cfg"))
        .def("ready", &PoseDDSPublisher::ready)
        .def("descriptor_json", &PoseDDSPublisher::descriptor_json)
        .def("publish", &PoseDDSPublisher::publish,
             py::arg("stamp_ms"),
             py::arg("x"), py::arg("y"), py::arg("yaw"),
             py::arg("vx"), py::arg("vy"), py::arg("omega"),
             py::arg("adv"), py::arg("side"), py::arg("rot"),
             py::arg("sx"), py::arg("sy"), py::arg("syaw"),
             py::arg("svx"), py::arg("svy"),
             py::arg("confidence"));
}
