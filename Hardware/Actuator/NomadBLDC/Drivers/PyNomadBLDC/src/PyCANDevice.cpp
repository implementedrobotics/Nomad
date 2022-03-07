// Primary Includes
#include <CAN/CANDevice.h>
// #include <CAN/PCANDevice.h>

// PyBind Include
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


namespace py = pybind11;


PYBIND11_MODULE(module_name, m)
{

    m.doc() = "Python Wrapper for Nomad CAN Interfaces";

    py::class_<CANDevice::Config_t>(m, "can_config")
        .def_readwrite("id", &CANDevice::Config_t::id)
        .def_readwrite("bitrate", &CANDevice::Config_t::bitrate)
        .def_readwrite("d_bitrate", &CANDevice::Config_t::d_bitrate)
        .def_readwrite("mode_fd", &CANDevice::Config_t::mode_fd)
        .def_readwrite("sample_point", &CANDevice::Config_t::sample_point)
        .def_readwrite("d_sample_point", &CANDevice::Config_t::d_sample_point)
        .def_readwrite("clock_freq", &CANDevice::Config_t::clock_freq)

        .def_readwrite("brp", &CANDevice::Config_t::brp)
        .def_readwrite("tq", &CANDevice::Config_t::tq)
        .def_readwrite("tseg1", &CANDevice::Config_t::tseg1)
        .def_readwrite("tseg2", &CANDevice::Config_t::tseg2)
        .def_readwrite("sjw", &CANDevice::Config_t::sjw)

        .def_readwrite("d_brp", &CANDevice::Config_t::d_brp)
        .def_readwrite("d_tseg1", &CANDevice::Config_t::d_tseg1)
        .def_readwrite("d_tseg2", &CANDevice::Config_t::d_tseg2)
        .def_readwrite("d_sjw", &CANDevice::Config_t::d_sjw);

    py::class_<CANDevice::CAN_msg_t>(m, "can_msg")
        .def(py::init<>())
        .def_readwrite("id", &CANDevice::CAN_msg_t::id)
        .def_readwrite("length", &CANDevice::CAN_msg_t::length)
        .def_readwrite("data", &CANDevice::CAN_msg_t::data);


    // py::class_<CANDevice>(m, "PyCANDevice")
    //     .def("clear_filters", &CANDevice::ClearFilters);

    // py::class_<PCANDevice, CANDevice>(m, "PyPCANDevice")
    //     .def(py::init<>());
}
