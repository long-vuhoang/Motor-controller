/*
 * @brief CanDevice implementation.
 */

#include "internal/can_device.h"

#include <chrono>

#include "internal/common_utils.h"
#include "internal/robstride.h"

using namespace std::chrono_literals;

namespace xyber {

// ── Construction / destruction ───────────────────────────────────────────────

CanDevice::CanDevice(std::string name,
                     std::array<std::string, MAX_CAN_BUSES> interfaces)
    : name_(std::move(name)) {
  for (uint8_t i = 0; i < MAX_CAN_BUSES; ++i) {
    if (!interfaces[i].empty()) {
      buses_[i] = std::make_unique<CanBus>(interfaces[i]);
    }
    /* Buses with empty interface string are left as nullptr (unused). */
  }
  LOG_DEBUG("[CanDevice %s] Created.", name_.c_str());
}

CanDevice::~CanDevice() {
  Stop();
  /* buses_ unique_ptrs → CanBus dtors → delete all owned actuators. */
}

// ── Actuator registration ────────────────────────────────────────────────────

bool CanDevice::AttachActuator(uint8_t bus_idx, ActuatorType type,
                               const std::string& actr_name, uint8_t can_id) {
  if (is_running_) {
    LOG_ERROR("[CanDevice %s] Cannot attach actuator while running. Stop first.",
              name_.c_str());
    return false;
  }
  if (bus_idx >= MAX_CAN_BUSES) {
    LOG_ERROR("[CanDevice %s] bus_idx %u out of range (0–%u).",
              name_.c_str(), bus_idx, MAX_CAN_BUSES - 1);
    return false;
  }
  if (!buses_[bus_idx]) {
    LOG_ERROR("[CanDevice %s] Bus %u has no interface configured.",
              name_.c_str(), bus_idx);
    return false;
  }
  if (actuator_bus_map_.count(actr_name)) {
    LOG_ERROR("[CanDevice %s] Actuator '%s' is already attached.",
              name_.c_str(), actr_name.c_str());
    return false;
  }

  /*
   * Allocate the concrete actuator.  CtrlChannel is a legacy field kept for
   * binary compatibility with Actuator base; it has no routing meaning here
   * (CanBus uses can_id directly).
   */
  Actuator* actr = nullptr;
  switch (type) {
    case ActuatorType::Robstride_00:
      actr = new xyber::RobstrideMotor(
          type, actr_name, can_id);
      break;
    case ActuatorType::Robstride_02:
      actr = new xyber::RobstrideMotor(
          type, actr_name, can_id);
      break;
    default:
      LOG_ERROR("[CanDevice %s] Unsupported actuator type %d for '%s'.",
                name_.c_str(), static_cast<int>(type), actr_name.c_str());
      return false;
  }

  /*
   * RegisterActuator takes ownership: if it fails it deletes actr.
   */
  if (!buses_[bus_idx]->RegisterActuator(actr)) {
    return false;  /* actr already deleted inside RegisterActuator */
  }

  actuator_bus_map_[actr_name] = buses_[bus_idx].get();

  LOG_DEBUG("[CanDevice %s] Attached actuator '%s' (can_id=%u) on bus[%u]=%s.",
            name_.c_str(), actr_name.c_str(), can_id, bus_idx,
            buses_[bus_idx]->GetInterface().c_str());
  return true;
}

// ── Life-cycle ────────────────────────────────────────────────────────────────

bool CanDevice::Start(uint64_t cycle_ns, int rt_priority, int bind_cpu) {
  if (is_running_) {
    LOG_WARN("[CanDevice %s] Already running.", name_.c_str());
    return true;
  }
  is_running_ = true;
  control_loop_thread_ = std::thread(&CanDevice::ControlLoop, this,
                                     cycle_ns, rt_priority, bind_cpu);
  LOG_INFO("[CanDevice %s] Control loop started (cycle=%lu ns, rt=%d, cpu=%d).",
           name_.c_str(), cycle_ns, rt_priority, bind_cpu);
  return true;
}

void CanDevice::Stop() {
  if (!is_running_) return;
  is_running_ = false;
  if (control_loop_thread_.joinable()) control_loop_thread_.join();
  LOG_INFO("[CanDevice %s] Stopped.", name_.c_str());
}

// ── Control loop ─────────────────────────────────────────────────────────────

void CanDevice::ControlLoop(uint64_t cycle_ns, int rt_priority, int bind_cpu) {
  /*
   * Apply realtime scheduling/affinity via the existing utility.
   * Thread name is truncated to 15 chars by the kernel.
   */
  xyber_utils::SetRealTimeThread(
      pthread_self(),
      ("can_" + name_).substr(0, 15),
      rt_priority,
      bind_cpu);

  auto next_wake = std::chrono::steady_clock::now();

  while (is_running_) {
    next_wake += std::chrono::nanoseconds(cycle_ns);

    /* Fire TX on every active bus. */
    for (auto& bus : buses_) {
      if (bus) bus->TransmitAll();
    }

    std::this_thread::sleep_until(next_wake);
  }
}

// ── Internal helper ───────────────────────────────────────────────────────────

CanBus* CanDevice::FindBus(const std::string& name) {
  auto it = actuator_bus_map_.find(name);
  return (it != actuator_bus_map_.end()) ? it->second : nullptr;
}

// ── Enable / Disable ─────────────────────────────────────────────────────────

bool CanDevice::EnableAllActuator() {
  for (auto& bus : buses_) {
    if (bus && !bus->EnableAll()) return false;
  }
  return true;
}

bool CanDevice::EnableActuator(const std::string& name) {
  auto* bus = FindBus(name);
  return bus && bus->EnableActuator(name);
}

bool CanDevice::DisableAllActuator() {
  for (auto& bus : buses_) {
    if (bus && !bus->DisableAll()) return false;
  }
  return true;
}

bool CanDevice::DisableActuator(const std::string& name) {
  auto* bus = FindBus(name);
  return bus && bus->DisableActuator(name);
}

// ── Config ────────────────────────────────────────────────────────────────────

void CanDevice::ClearError(const std::string& name) {
  if (auto* bus = FindBus(name)) bus->ClearError(name);
}

void CanDevice::SetHomingPosition(const std::string& name) {
  if (auto* bus = FindBus(name)) bus->SetHomingPosition(name);
}

void CanDevice::SaveConfig(const std::string& name) {
  if (auto* bus = FindBus(name)) bus->SaveConfig(name);
}

// ── Getters ───────────────────────────────────────────────────────────────────

float CanDevice::GetTempure(const std::string& name) {
  auto* bus = FindBus(name);
  return bus ? bus->GetTempure(name) : 0.0f;
}

std::string CanDevice::GetErrorString(const std::string& name) {
  auto* bus = FindBus(name);
  return bus ? bus->GetErrorString(name) : "";
}

ActuatorState CanDevice::GetPowerState(const std::string& name) {
  auto* bus = FindBus(name);
  return bus ? bus->GetPowerState(name) : STATE_DISABLE;
}

bool CanDevice::SetMode(const std::string& name, ActuatorMode mode) {
  auto* bus = FindBus(name);
  return bus && bus->SetMode(name, mode);
}

ActuatorMode CanDevice::GetMode(const std::string& name) {
  auto* bus = FindBus(name);
  return bus ? bus->GetMode(name) : MODE_CURRENT;
}

float CanDevice::GetEffort(const std::string& name) {
  auto* bus = FindBus(name);
  return bus ? bus->GetEffort(name) : 0.0f;
}

float CanDevice::GetVelocity(const std::string& name) {
  auto* bus = FindBus(name);
  return bus ? bus->GetVelocity(name) : 0.0f;
}

float CanDevice::GetPosition(const std::string& name) {
  auto* bus = FindBus(name);
  return bus ? bus->GetPosition(name) : 0.0f;
}

// ── Setters ───────────────────────────────────────────────────────────────────

void CanDevice::SetEffort(const std::string& name, float cur) {
  if (auto* bus = FindBus(name)) bus->SetEffort(name, cur);
}

void CanDevice::SetVelocity(const std::string& name, float vel) {
  if (auto* bus = FindBus(name)) bus->SetVelocity(name, vel);
}

void CanDevice::SetPosition(const std::string& name, float pos) {
  if (auto* bus = FindBus(name)) bus->SetPosition(name, pos);
}

void CanDevice::SetMitParam(const std::string& name, MitParam param) {
  if (auto* bus = FindBus(name)) bus->SetMitParam(name, param);
}

void CanDevice::SetMitCmd(const std::string& name, float pos, float vel,
                          float effort, float kp, float kd) {
  if (auto* bus = FindBus(name)) bus->SetMitCmd(name, pos, vel, effort, kp, kd);
}

}  // namespace xyber