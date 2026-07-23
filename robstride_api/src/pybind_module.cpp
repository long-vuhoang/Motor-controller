#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>
#include "xyber_controller.h"
#include "common_type.h"

namespace py = pybind11;
using namespace xyber;

PYBIND11_MODULE(xyber_py, m) {
    m.doc() = "XyberController Python SDK - SocketCAN binding";

    // ── ActuatorType enum ──────────────────────────────────────────────────
    py::enum_<ActuatorType>(m, "ActuatorType")
        .value("Robstride_00", ActuatorType::Robstride_00)
        .value("Robstride_02", ActuatorType::Robstride_02)
        .value("Robstride_05", ActuatorType::Robstride_05)
        .export_values();

    // ── ActuatorState enum ─────────────────────────────────────────────────
    // FIX: .cpp dùng STATE_DISABLE (C-style), không phải ActuatorState::DISABLED
    py::enum_<ActuatorState>(m, "ActuatorState")
        .value("STATE_DISABLE", STATE_DISABLE)
        .value("STATE_ENABLE",  STATE_ENABLE)
        .export_values();

    // ── ActuatorMode enum ──────────────────────────────────────────────────
    // FIX: .cpp dùng MODE_CURRENT làm default → cần expose đủ các mode
    py::enum_<ActuatorMode>(m, "ActuatorMode")
        .value("MODE_MIT",     MODE_MIT)
        .value("MODE_CURRENT", MODE_CURRENT)
        .export_values();

    // ── XyberController ────────────────────────────────────────────────────
    // FIX: py::nodelete vì destructor gọi Stop() + delete devices
    //      Python không được phép free singleton này
    py::class_<XyberController,
               std::unique_ptr<XyberController, py::nodelete>>(m, "XyberController")

        .def_static("get_instance", &XyberController::GetInstance,
            py::return_value_policy::reference,
            "Trả về singleton instance (lazy init).")

        .def("get_version", &XyberController::GetVersion,
            "Trả về version string 'major.minor.patch'.")

        // ── Configuration ──────────────────────────────────────────────────

        .def("create_device",
            // FIX: Wrap vì Python list không tự cast sang std::array<string,4>
            //      Pad "" cho bus không dùng thay vì báo lỗi
            [](XyberController& self,
               const std::string& name,
               const std::vector<std::string>& ifaces) -> bool {
                if (ifaces.size() > CONTROLLER_MAX_CAN_BUSES) {
                    throw std::invalid_argument(
                        "interfaces: tối đa " +
                        std::to_string(CONTROLLER_MAX_CAN_BUSES) + " bus.");
                }
                std::array<std::string, CONTROLLER_MAX_CAN_BUSES> arr{};
                for (size_t i = 0; i < ifaces.size(); ++i) arr[i] = ifaces[i];
                return self.CreateDevice(name, arr);
            },
            py::arg("name"),
            py::arg("interfaces"),
            "Tạo CanDevice với tối đa 4 SocketCAN interface.\n"
            "Bus không dùng tự động điền ''. Gọi trước start().\n"
            "Ví dụ: create_device('arm', ['can0', 'can1'])")

        .def("attach_actuator", &XyberController::AttachActuator,
            py::arg("device_name"),
            py::arg("bus_idx"),
            py::arg("type"),
            py::arg("actr_name"),
            py::arg("can_id"),
            "Gắn actuator vào bus_idx (0-3) của device.\n"
            "Trả về False nếu device không tồn tại hoặc tên trùng.\n"
            "Ví dụ: attach_actuator('arm', 0, ActuatorType.Robstride_02, 'j1', 1)")

        .def("set_realtime", &XyberController::SetRealtime,
            py::arg("rt_priority"),
            py::arg("bind_cpu") = -1,
            "Đặt SCHED_FIFO priority + CPU affinity (gọi trước start()).\n"
            "rt_priority 0-99 | -1 = scheduler thường.\n"
            "bind_cpu: core index | -1 = không pin.\n"
            "⚠ Yêu cầu CAP_SYS_NICE hoặc sudo.\n"
            "FIX: Trả về False ngay nếu gọi sau khi đã start().")

        .def("start", &XyberController::Start,
            py::arg("cycle_ns") = 1'000'000,
            "Khởi động tất cả device control-loop threads.\n"
            "Nếu bất kỳ device nào fail → rollback Stop() toàn bộ.\n"
            "cycle_ns: chu kỳ TX (ns), mặc định 1 ms.")

        .def("stop", &XyberController::Stop,
            "Dừng toàn bộ threads và join. Idempotent (gọi nhiều lần an toàn).")

        // ── Enable / Disable ───────────────────────────────────────────────

        .def("enable_all_actuator",  &XyberController::EnableAllActuator)
        .def("disable_all_actuator", &XyberController::DisableAllActuator)

        .def("enable_actuator",  &XyberController::EnableActuator,
            py::arg("name"))
        .def("disable_actuator", &XyberController::DisableActuator,
            py::arg("name"))

        // ── Getters ────────────────────────────────────────────────────────
        // FIX: Tất cả getter trả về giá trị mặc định (0 / STATE_DISABLE)
        //      nếu tên actuator không tìm thấy - không raise exception

        .def("get_position", &XyberController::GetPosition,
            py::arg("name"), "Vị trí (rad). Trả về 0.0 nếu không tìm thấy.")

        .def("get_velocity", &XyberController::GetVelocity,
            py::arg("name"), "Vận tốc (rad/s). Trả về 0.0 nếu không tìm thấy.")

        .def("get_effort", &XyberController::GetEffort,
            py::arg("name"), "Torque (Nm) hoặc norm 0-1. Trả về 0.0 nếu không tìm thấy.")

        .def("get_tempure", &XyberController::GetTempure,
            py::arg("name"), "Nhiệt độ (°C). Trả về 0.0 nếu không tìm thấy.")

        .def("get_power_state", &XyberController::GetPowerState,
            py::arg("name"), "Trả về ActuatorState. Default: STATE_DISABLE.")

        .def("get_mode", &XyberController::GetMode,
            py::arg("name"), "Trả về ActuatorMode. Default: MODE_CURRENT.")

        // ── Setters ────────────────────────────────────────────────────────
        .def("set_homing_position", 
             &XyberController::SetHomingPosition, 
             py::arg("name"), "Đặt vị trí cơ học hiện tại làm mốc zero tham chiếu (COMM_SET_ZERO).")
        // ── MIT ────────────────────────────────────────────────────────────
        .def("set_mit_cmd", &XyberController::SetMitCmd,
            py::arg("name"),
            py::arg("pos"),
            py::arg("vel")    = 0.0f,
            py::arg("effort") = 0.0f,
            py::arg("kp")     = 0.0f,
            py::arg("kd")     = 0.0f,
            "Gửi lệnh MIT. Silent noop nếu tên không tồn tại.\n"
            "PowerFlowL/OmniPicker: chỉ dùng pos + effort.\n"
            "Ví dụ: set_mit_cmd('j1', 1.57, 0, 0, 0.9, 0.2)");
}