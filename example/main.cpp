/**
 * @file example_usage.cpp
 * @brief Ví dụ sử dụng XyberController với SocketCAN.
 *
 * Hardware giả định:
 *   1 thiết bị vật lý, 4 CAN bus (can0–can3)
 *   Bus 0: 3 động cơ PowerFlowR86  (can_id 1, 2, 3)
 *   Bus 1: 2 động cơ PowerFlowR52  (can_id 1, 2)
 *   Bus 2: 1 động cơ PowerFlowL28  (can_id 1)
 *   Bus 3: 1 gripper OmniPicker     (can_id 1)
 *
 * So sánh với API EtherCAT cũ:
 * ─────────────────────────────
 *  EtherCAT (cũ)                       SocketCAN (mới)
 *  ─────────────────────────────────── ─────────────────────────────────────
 *  CreateDcu("dcu1", ecat_id=1)        CreateDevice("dev1", {"can0","can1","can2","can3"})
 *  AttachActuator("dcu1",CTRL_CH1,...) AttachActuator("dev1", bus_idx=0, ...)
 *  Start("eth0", 1e6, true)            Start(1'000'000)   ← cycle_ns only
 */

#include "xyber_controller.h"

#include <cstdint>
#include <iostream>
#include <thread>

using namespace xyber;
using namespace std::chrono_literals;

int main() {
  auto* ctrl = XyberController::GetInstance();

  // ── 1. Tạo thiết bị với 4 CAN interface ──────────────────────────────────
  ctrl->CreateDevice("arm", {"can0", "can1", "can2", "can3"});

  // ── 2. Gắn động cơ: (device, bus_idx, type, name, can_id) ────────────────
  // Bus 0 – tối đa 3 động cơ
  ctrl->AttachActuator("arm", 0, ActuatorType::Robstride_00, "a", 1);
  ctrl->AttachActuator("arm", 0, ActuatorType::Robstride_02, "b", 2);
  ctrl->AttachActuator("arm", 1, ActuatorType::Robstride_00, "c", 1);
  ctrl->AttachActuator("arm", 1, ActuatorType::Robstride_02, "d", 2);
  ctrl->AttachActuator("arm", 2, ActuatorType::Robstride_00, "e", 3);
  ctrl->AttachActuator("arm", 2, ActuatorType::Robstride_00, "f", 4);
  ctrl->AttachActuator("arm", 3, ActuatorType::Robstride_00, "g", 3);
  ctrl->AttachActuator("arm", 3, ActuatorType::Robstride_00, "h", 4);

  // ── 3. Tuỳ chọn: đặt realtime ────────────────────────────────────────────
  ctrl->SetRealtime(/*rt_priority=*/80, /*bind_cpu=*/3);

  // ── 4. Start: khởi động control-loop threads ──────────────────────────────
  if (!ctrl->Start(/*cycle_ns=*/1'000'000)) {  // 1 ms
    return -1;
  }

  // ── 5. Enable ─────────────────────────────────────────────────────────────
  ctrl->EnableAllActuator();
  // Hoặc từng actuator:
  // ctrl->EnableActuator("shoulder_pitch");

  // ── 6. MIT control loop ───────────────────────────────────────────────────
  float dt = 0;
  float pos_begin = ctrl->GetPosition("a");
  for (size_t i = 0; i < 100 * 30; i++) {

    // Set target position using MIT mode
    double pos_cmd = pos_begin + 2 * sin(dt);
    ctrl->SetMitCmd("a", pos_cmd, 0, 0, 0.9, 0.2);
    ctrl->SetMitCmd("b", pos_cmd, 0, 0, 0.9, 0.2);
    ctrl->SetMitCmd("c", pos_cmd, 0, 0, 0.9, 0.2);
    ctrl->SetMitCmd("d", pos_cmd, 0, 0, 0.9, 0.2);
    ctrl->SetMitCmd("e", pos_cmd, 0, 0, 0.9, 0.2);
    ctrl->SetMitCmd("f", pos_cmd, 0, 0, 0.9, 0.2);
    ctrl->SetMitCmd("g", pos_cmd, 0, 0, 0.9, 0.2);
    ctrl->SetMitCmd("h", pos_cmd, 0, 0, 0.9, 0.2);

    // read current position
    float pos_now = ctrl->GetPosition("a");
    //std::cout << "Position: Cmd " << pos_cmd << " Now " << pos_now << std::endl;

    // phase control
    dt += 0.01;
    if (dt >= 6.28) {
      dt = 0.0;
    }
    std::this_thread::sleep_for(10ms);
  }

  // ── 7. Shutdown ───────────────────────────────────────────────────────────
  ctrl->DisableAllActuator();
  ctrl->Stop();
  return 0;
}