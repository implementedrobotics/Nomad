// Primary Includes
#include <CAN/CANDevice.h>
#include <CAN/PCANDevice.h>
#include <CAN/SocketCANDevice.h>

// Nomad BLDC CAN Driver
#include <NomadBLDC/NomadBLDC.h>
#include <NomadBLDC/Registers.h>

// PyBind Include
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(nomad, m)
{

    m.doc() = "Python Wrapper for Nomad CAN Interfaces";

    py::class_<CANDevice::Config_t>(m, "can_config")
        .def(py::init<>())
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

    // py::class_<CANDevice::CAN_msg_t>(m, "can_msg")
    //     .def(py::init<>())
    //     .def_readwrite("id", &CANDevice::CAN_msg_t::id)
    //     .def_readwrite("length", &CANDevice::CAN_msg_t::length)
    //     .def_readwrite("data", &CANDevice::CAN_msg_t::data);

    py::class_<Register>(m, "register")
        .def(py::init<>())
        .def_readwrite("address", &Register::address)
        .def_readwrite("size", &Register::size)
        .def_readwrite("data", &Register::data);

    py::class_<CANDevice>(m, "can_device")
        .def("open", &CANDevice::Open)
        .def("close", &CANDevice::Close)
        .def("add_filter", &CANDevice::AddFilter)
        .def("clear_filters", &CANDevice::ClearFilters)
        .def("send", static_cast<bool (CANDevice::*)(CANDevice::CAN_msg_t &)>(&CANDevice::Send))
        .def("send", static_cast<bool (CANDevice::*)(uint32_t, uint8_t *, uint16_t)>(&CANDevice::Send))
        .def("receive", &CANDevice::Receive)
        .def("status", &CANDevice::Status);

    py::class_<PCANDevice, CANDevice>(m, "pcan_device")
        .def(py::init<>());

    py::class_<SocketCANDevice, CANDevice>(m, "socket_can_device")
        .def(py::init<>());

    py::class_<NomadBLDC>(m, "nomad_bldc")
        // Constructors
        .def(py::init<>())
        .def(py::init<int, int, CANDevice *>())

        // Transports
        .def("set_transport", &NomadBLDC::SetTransport)
        .def("connect", &NomadBLDC::Connect)
        .def("disconnect", &NomadBLDC::Disconnect)
        .def("reset", &NomadBLDC::Reset)

        // Servo Config
        .def("set_name", &NomadBLDC::SetName)
        .def("name", &NomadBLDC::GetName)
        .def("get_servo_id", &NomadBLDC::GetServoId)
        .def("zero_output", &NomadBLDC::ZeroOutput)
        .def("set_max_torque", &NomadBLDC::SetMaxTorque)
        .def("get_max_torque", &NomadBLDC::GetMaxTorque)
        .def("set_max_current", &NomadBLDC::SetMaxCurrent)
        .def("get_max_current", &NomadBLDC::GetMaxCurrent)
        .def("save_config", &NomadBLDC::SaveConfig)

        // Control
        .def("set_control_mode", &NomadBLDC::SetControlMode)
        .def("closed_loop_torque_command", &NomadBLDC::ClosedLoopTorqueCommand)
        .def("sync", &NomadBLDC::Sync)

        // Servo State
        .def("get_position", &NomadBLDC::GetPosition)
        .def("get_velocity", &NomadBLDC::GetVelocity)
        .def("get_torque", &NomadBLDC::GetTorque)

        // Registers
        .def("read_registers", &NomadBLDC::ReadRegisters)
        .def("write_registers", &NomadBLDC::WriteRegisters)
        .def("execute_registers", &NomadBLDC::ExecuteRegisters)

        // Debug
        .def("print_state", &NomadBLDC::PrintState);
}
