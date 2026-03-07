/*
 * @brief XyberController implementation (SocketCAN build).
 */

#include "xyber_controller.h"

#include <unordered_map>

#include "common_type.h"
#include "internal/can_device.h"
#include "internal/version.h"

namespace xyber {

// ── Singleton state ───────────────────────────────────────────────────────────

XyberController* XyberController::instance_ = nullptr;

/*
 * Module-level storage (hidden from public API, same pattern as original).
 * device_map_   : name → owned CanDevice*
 * actr_dev_map_ : actuator name → non-owning CanDevice*
 */
static std::unordered_map<std::string, CanDevice*> device_map_;
static std::unordered_map<std::string, CanDevice*> actr_dev_map_;

// ── Construction / destruction ────────────────────────────────────────────────

XyberController::XyberController() {
  LOG_INFO("XyberController Ver %d.%d.%d (SocketCAN) – compiled %s %s.",
           MAIN_VERSION, SUB_VERSION, PATCH_VERSION, __DATE__, __TIME__);
}

XyberController::~XyberController() {
  Stop();
  for (auto& [name, dev] : device_map_) delete dev;
  device_map_.clear();
  actr_dev_map_.clear();
}

XyberController* XyberController::GetInstance() {
  if (!instance_) instance_ = new XyberController();
  return instance_;
}

std::string XyberController::GetVersion() {
  return std::to_string(MAIN_VERSION) + "." +
         std::to_string(SUB_VERSION)  + "." +
         std::to_string(PATCH_VERSION);
}

// ── Configuration ─────────────────────────────────────────────────────────────

bool XyberController::CreateDevice(
    const std::string& name,
    std::array<std::string, CONTROLLER_MAX_CAN_BUSES> interfaces) {

  if (device_map_.count(name)) {
    LOG_ERROR("Device '%s' already exists.", name.c_str());
    return false;
  }
  device_map_[name] = new CanDevice(name, interfaces);
  LOG_INFO("Device '%s' created (buses: %s | %s | %s | %s).",
           name.c_str(),
           interfaces[0].c_str(), interfaces[1].c_str(),
           interfaces[2].c_str(), interfaces[3].c_str());
  return true;
}

bool XyberController::AttachActuator(const std::string& device_name,
                                     uint8_t            bus_idx,
                                     ActuatorType       type,
                                     const std::string& actr_name,
                                     uint8_t            can_id) {
  auto it = device_map_.find(device_name);
  if (it == device_map_.end()) {
    LOG_ERROR("Device '%s' not found. Create it first.", device_name.c_str());
    return false;
  }
  if (actr_dev_map_.count(actr_name)) {
    LOG_ERROR("Actuator '%s' is already attached.", actr_name.c_str());
    return false;
  }
  if (!it->second->AttachActuator(bus_idx, type, actr_name, can_id)) {
    return false;
  }
  actr_dev_map_[actr_name] = it->second;
  return true;
}

bool XyberController::SetRealtime(int rt_priority, int bind_cpu) {
  if (is_running_) {
    LOG_WARN("Cannot change realtime settings while running.");
    return false;
  }
  rt_priority_ = rt_priority;
  bind_cpu_    = bind_cpu;
  return true;
}

// ── Life-cycle ────────────────────────────────────────────────────────────────

bool XyberController::Start(uint64_t cycle_ns) {
  if (is_running_) {
    LOG_WARN("XyberController is already running.");
    return true;
  }
  if (device_map_.empty()) {
    LOG_ERROR("No devices created. Call CreateDevice() first.");
    return false;
  }

  for (auto& [name, dev] : device_map_) {
    if (!dev->Start(cycle_ns, rt_priority_, bind_cpu_)) {
      LOG_ERROR("Failed to start device '%s'.", name.c_str());
      /* Roll back already-started devices. */
      for (auto& [n2, d2] : device_map_) d2->Stop();
      return false;
    }
  }

  is_running_ = true;
  LOG_INFO("XyberController started – %zu device(s), cycle %lu ns.",
           device_map_.size(), cycle_ns);
  return true;
}

void XyberController::Stop() {
  if (!is_running_) return;
  for (auto& [name, dev] : device_map_) dev->Stop();
  is_running_ = false;
  LOG_INFO("XyberController stopped.");
}

// ── Enable / Disable ──────────────────────────────────────────────────────────

bool XyberController::EnableAllActuator() {
  for (auto& [name, dev] : device_map_) {
    if (!dev->EnableAllActuator()) return false;
  }
  return true;
}

bool XyberController::EnableActuator(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) && it->second->EnableActuator(name);
}

bool XyberController::DisableAllActuator() {
  for (auto& [name, dev] : device_map_) {
    if (!dev->DisableAllActuator()) return false;
  }
  return true;
}

bool XyberController::DisableActuator(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) && it->second->DisableActuator(name);
}

// ── Getters ───────────────────────────────────────────────────────────────────

float XyberController::GetTempure(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) ? it->second->GetTempure(name) : 0.0f;
}

ActuatorState XyberController::GetPowerState(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) ? it->second->GetPowerState(name)
                                     : STATE_DISABLE;
}

ActuatorMode XyberController::GetMode(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) ? it->second->GetMode(name) : MODE_CURRENT;
}

float XyberController::GetEffort(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) ? it->second->GetEffort(name) : 0.0f;
}

float XyberController::GetVelocity(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) ? it->second->GetVelocity(name) : 0.0f;
}

float XyberController::GetPosition(const std::string& name) {
  auto it = actr_dev_map_.find(name);
  return (it != actr_dev_map_.end()) ? it->second->GetPosition(name) : 0.0f;
}

// ── MIT ───────────────────────────────────────────────────────────────────────

void XyberController::SetMitParam(const std::string& name, MitParam param) {
  auto it = actr_dev_map_.find(name);
  if (it != actr_dev_map_.end()) it->second->SetMitParam(name, param);
}

void XyberController::SetMitCmd(const std::string& name,
                                float pos, float vel, float effort,
                                float kp, float kd) {
  auto it = actr_dev_map_.find(name);
  if (it != actr_dev_map_.end())
    it->second->SetMitCmd(name, pos, vel, effort, kp, kd);
}

}  // namespace xyber