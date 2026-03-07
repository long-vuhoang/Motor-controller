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
using namespace xyber;

int main() {
  auto* ctrl = XyberController::GetInstance();

  // ── 1. Tạo thiết bị với 4 CAN interface ──────────────────────────────────
  ctrl->CreateDevice("arm", {"can0", "can1", "can2", "can3"});

  // ── 2. Gắn động cơ: (device, bus_idx, type, name, can_id) ────────────────
  // Bus 0 – tối đa 3 động cơ
  ctrl->AttachActuator("arm", 0, ActuatorType::Robstride_02, "shoulder_pitch", 1);
  ctrl->AttachActuator("arm", 0, ActuatorType::Robstride_02, "shoulder_roll",  2);
  ctrl->AttachActuator("arm", 0, ActuatorType::Robstride_02, "shoulder_yaw",   3);

  // Bus 1 – tối đa 3 động cơ
  ctrl->AttachActuator("arm", 1, ActuatorType::Robstride_02, "elbow_pitch",    1);
  ctrl->AttachActuator("arm", 1, ActuatorType::Robstride_02, "elbow_yaw",      2);

  // Bus 2
  ctrl->AttachActuator("arm", 2, ActuatorType::Robstride_00, "wrist_roll",     1);

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
  for (int i = 0; i < 1000; ++i) {
    float pos = ctrl->GetPosition("shoulder_pitch");
    ctrl->SetMitCmd("shoulder_pitch",
                    /*pos=*/0.0f, /*vel=*/0.0f, /*effort=*/0.0f,
                    /*kp=*/10.0f, /*kd=*/1.0f);
    // ...
  }

  // ── 7. Shutdown ───────────────────────────────────────────────────────────
  ctrl->DisableAllActuator();
  ctrl->Stop();
  return 0;
}