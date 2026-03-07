/*
 * @brief CanBus implementation – Robstride EFF (29-bit extended) protocol.
 *
 * Key differences from the previous SFF (standard 11-bit) implementation:
 *
 * 1. SLOT SIZE:  8 bytes (data only)  →  12 bytes (CanFrame = can_id + data).
 *    Actuators write the FULL CanFrame (including the EFF can_id) into their
 *    send slot.  TransmitAll() reads and transmits it verbatim.
 *
 * 2. TX CAN ID:  set by the actuator (includes CAN_EFF_FLAG + comm_type +
 *    extra data).  CanBus never constructs the TX can_id itself.
 *
 * 3. RX KEY EXTRACTOR:  Robstride feedback has motor_id at bits [15:8].
 *    The extractor (frame.can_id >> 8) & 0xFF maps each feedback frame to
 *    the correct per-actuator callback.
 *
 * 4. RX CALLBACK:  stores the full CanFrame (can_id + data) in recv_slot_ so
 *    the actuator's ParseFeedback() can decode both the status bits in the
 *    can_id and the data payload.
 *
 * Command routing (unchanged logic, new slot semantics):
 * ───────────────────────────────────────────────────────
 *  PERIODIC  : actuator writes full CanFrame → TransmitAll() sends each cycle.
 *  ONE-SHOT  : SendOnce() → write → copy to local frame → zero slot → transmit.
 */

#include "internal/can_bus.h"

#include <chrono>
#include <cstring>
#include <functional>
#include <thread>

#define COM_TIMEOUT_MS 1000

using namespace std::chrono_literals;

namespace xyber {

// ── Construction / destruction ───────────────────────────────────────────────

CanBus::CanBus(const std::string& interface) : interface_(interface) {
  send_buf_.fill(0);
  recv_buf_.fill(0);

  socket_can_ = SocketCAN::get(interface_);

  /*
   * Set the RX key extractor for Robstride EFF feedback frames.
   *
   * Robstride feedback (comm_type=0x02) CAN ID layout:
   *   bits [31:24] = 0x02
   *   bits [23:8]  = extra_data  (status flags, device_id in [7:0])
   *   bits [15:8]  = motor_id    ← this is the dispatch key
   *   bits [7:0]   = host_id (0x00 in feedback)
   *
   * The RX callback is registered with key = motor_id (1–3).
   * Extractor: (frame.can_id >> 8) & 0xFF  → motor_id.
   *
   * Note: SocketCAN::get() returns a shared singleton per interface.
   * Setting the extractor here affects all users of the same interface.
   * Since all buses in this codebase are Robstride-only, the extractor is
   * consistent and idempotent across multiple CanBus instances on the same
   * interface string.
   */
  socket_can_->set_key_extractor([](const can_frame& f) -> CanCbkId {
    return static_cast<CanCbkId>((f.can_id >> 8) & 0xFF);
  });

  LOG_DEBUG("[CanBus %s] Created (EFF/Robstride mode).", interface_.c_str());
}

CanBus::~CanBus() {
  for (auto& [name, actr] : actuator_by_name_) {
    delete actr;
  }
}

// ── Registration ─────────────────────────────────────────────────────────────

bool CanBus::RegisterActuator(Actuator* actr) {
  if (!actr) return false;

  const uint8_t id = actr->GetId();

  if (id == 0 || id > MAX_ACTUATORS_PER_BUS) {
    LOG_ERROR("[CanBus %s] Actuator '%s' invalid CAN id %u (range 1–%u).",
              interface_.c_str(), actr->GetName().c_str(), id,
              MAX_ACTUATORS_PER_BUS);
    delete actr;
    return false;
  }
  if (actuator_by_id_.count(id)) {
    LOG_ERROR("[CanBus %s] CAN id %u already occupied.", interface_.c_str(), id);
    delete actr;
    return false;
  }
  if (actuator_by_name_.count(actr->GetName())) {
    LOG_ERROR("[CanBus %s] Actuator name '%s' already registered.",
              interface_.c_str(), actr->GetName().c_str());
    delete actr;
    return false;
  }

  /*
   * SetDataFiled() points the actuator's send_slot_ / recv_slot_ pointers
   * directly at its 12-byte CanFrame slot in our send_buf_ / recv_buf_.
   * Offset = (id-1) * sizeof(CanFrame) — computed inside the actuator.
   */
  actr->SetDataFiled(send_buf_.data(), recv_buf_.data());
  actuator_by_name_[actr->GetName()] = actr;
  actuator_by_id_[id]                = actr;

  /*
   * Register a DEDICATED RX callback for this motor's CAN id.
   *
   * The key extractor returns (frame.can_id >> 8) & 0xFF, so this callback
   * fires when a Robstride feedback frame with motor_id == id arrives.
   *
   * The callback writes the FULL CanFrame (can_id + data) into recv_slot_
   * so that RobstrideMotor::ParseFeedback() can read both the status bits
   * from the extended ID and the position/velocity/torque/temperature from
   * the data payload.
   */
  const size_t slot_offset = sizeof(CanFrame) * static_cast<size_t>(id - 1);
  socket_can_->add_can_callback(
      [this, slot_offset](const can_frame& frame) {
        std::lock_guard<std::mutex> lock(recv_mtx_);
        CanFrame* slot = reinterpret_cast<CanFrame*>(
            recv_buf_.data() + slot_offset);
        slot->can_id = frame.can_id;   /* includes EFF flag + status bits */
        std::memcpy(slot->data, frame.data, CAN_MAX_DLEN);
      },
      static_cast<CanCbkId>(id));

  LOG_DEBUG("[CanBus %s] Registered '%s' at CAN id %u (Robstride %s).",
            interface_.c_str(), actr->GetName().c_str(), id,
            actr->GetType() == ActuatorType::Robstride_00 ? "00" : "02");
  return true;
}

Actuator* CanBus::GetActuator(const std::string& name) {
  auto it = actuator_by_name_.find(name);
  return (it != actuator_by_name_.end()) ? it->second : nullptr;
}

bool CanBus::HasActuator(const std::string& name) const {
  return actuator_by_name_.count(name) > 0;
}

// ── Periodic TX: called every cycle by CanDevice control loop ────────────────

void CanBus::TransmitAll() {
  if (actuator_by_id_.empty()) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  for (const auto& [id, actr] : actuator_by_id_) {
    const CanFrame* cf = reinterpret_cast<const CanFrame*>(
        send_buf_.data() + sizeof(CanFrame) * static_cast<size_t>(id - 1));

    /*
     * Skip if the slot has not been written yet (can_id == 0).
     * This happens before the first SetMitCmd() / SetVelocity() call.
     * Avoids transmitting a malformed frame with can_id=0 on startup.
     */
    if (cf->can_id == 0) continue;

    can_frame frame{};
    frame.can_id  = cf->can_id;   /* EFF flag already set by RobstrideMotor */
    frame.can_dlc = CAN_MAX_DLEN; /* Robstride always uses 8-byte payload   */
    std::memcpy(frame.data, cf->data, CAN_MAX_DLEN);

    socket_can_->transmit(frame);
  }
}

// ── SendOnce: one-shot command transmission ───────────────────────────────────

void CanBus::SendOnce(Actuator* actr, const std::function<void()>& cmd_fn) {
  can_frame frame{};

  {
    std::lock_guard<std::mutex> lock(send_mtx_);

    cmd_fn();   /* step 1: actuator writes full CanFrame into its send slot */

    CanFrame* slot = reinterpret_cast<CanFrame*>(
        send_buf_.data() +
        sizeof(CanFrame) * static_cast<size_t>(actr->GetId() - 1));

    /* step 2: capture the CanFrame into a local struct can_frame */
    frame.can_id  = slot->can_id;
    frame.can_dlc = CAN_MAX_DLEN;
    std::memcpy(frame.data, slot->data, CAN_MAX_DLEN);

    /* step 3: zero the slot so TransmitAll() sends idle zeros next cycle */
    std::memset(slot, 0x00, sizeof(CanFrame));
  }

  socket_can_->transmit(frame);   /* step 4: fire exactly once */
}

// ── Enable / Disable ─────────────────────────────────────────────────────────

bool CanBus::EnableAll() {
  for (const auto& [name, actr] : actuator_by_name_) {
    if (!EnableActuator(name)) return false;
  }
  return true;
}

bool CanBus::DisableAll() {
  for (const auto& [name, actr] : actuator_by_name_) {
    if (!DisableActuator(name)) return false;
  }
  return true;
}

bool CanBus::EnableActuator(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return false;

  SendOnce(actr, [&]() { actr->RequestState(ActuatorState::STATE_ENABLE); });

  ActuatorState ret = ActuatorState::STATE_DISABLE;
  for (size_t i = 0; i < COM_TIMEOUT_MS / 10; ++i) {
    std::this_thread::sleep_for(10ms);
    std::lock_guard<std::mutex> lock(recv_mtx_);
    ret = actr->GetPowerState();
    if (ret == ActuatorState::STATE_ENABLE) break;
  }

  if (ret == ActuatorState::STATE_ENABLE) {
    LOG_DEBUG("[CanBus %s] '%s' enabled.", interface_.c_str(), name.c_str());
  } else {
    LOG_ERROR("[CanBus %s] '%s' enable timeout (no feedback).",
              interface_.c_str(), name.c_str());
  }
  return ret == ActuatorState::STATE_ENABLE;
}

bool CanBus::DisableActuator(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return false;

  SendOnce(actr, [&]() { actr->RequestState(ActuatorState::STATE_DISABLE); });

  ActuatorState ret = ActuatorState::STATE_ENABLE;
  for (size_t i = 0; i < COM_TIMEOUT_MS / 10; ++i) {
    std::this_thread::sleep_for(10ms);
    std::lock_guard<std::mutex> lock(recv_mtx_);
    ret = actr->GetPowerState();
    if (ret == ActuatorState::STATE_DISABLE) break;
  }
  return ret == ActuatorState::STATE_DISABLE;
}

// ── One-shot config commands ──────────────────────────────────────────────────

void CanBus::ClearError(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  SendOnce(actr, [&]() { actr->ClearError(); });
}

void CanBus::SetHomingPosition(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  SendOnce(actr, [&]() { actr->SetHomingPosition(); });
}

void CanBus::SaveConfig(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  /*
   * For Robstride, SaveConfig() is a no-op (motor firmware persists config
   * automatically).  SendOnce() will transmit a zeroed frame harmlessly.
   */
  SendOnce(actr, [&]() { actr->SaveConfig(); });
}

// ── SetMode (one-shot, mode tracked internally by actuator) ──────────────────

bool CanBus::SetMode(const std::string& name, ActuatorMode mode) {
  auto* actr = GetActuator(name);
  if (!actr) return false;

  /*
   * Protocol note: Robstride requires the motor to be disabled before changing
   * the run mode.  Recommended caller sequence:
   *   DisableActuator(name) → SetMode(name, mode) → EnableActuator(name)
   *
   * SetMode() here only writes the COMM_SET_PARAM frame for PARAM_RUN_MODE.
   * The actuator tracks current_mode_ internally; GetMode() returns the new
   * mode immediately after SetMode() returns (no round-trip verification).
   */
  SendOnce(actr, [&]() { actr->SetMode(mode); });

  /* Verify by reading back the internally tracked mode. */
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetMode() == mode;
}

// ── Getters ───────────────────────────────────────────────────────────────────

float CanBus::GetTempure(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return 0.0f;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetTempure();
}

std::string CanBus::GetErrorString(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return "";
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetErrorString();
}

ActuatorState CanBus::GetPowerState(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return ActuatorState::STATE_DISABLE;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetPowerState();
}

ActuatorMode CanBus::GetMode(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return ActuatorMode::MODE_MIT;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetMode();
}

float CanBus::GetEffort(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return 0.0f;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetEffort();
}

float CanBus::GetVelocity(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return 0.0f;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetVelocity();
}

float CanBus::GetPosition(const std::string& name) {
  auto* actr = GetActuator(name);
  if (!actr) return 0.0f;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetPosition();
}

// ── Periodic setters ──────────────────────────────────────────────────────────

void CanBus::SetEffort(const std::string& name, float cur) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetEffort(cur);
}

void CanBus::SetVelocity(const std::string& name, float vel) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetVelocity(vel);
}

void CanBus::SetPosition(const std::string& name, float pos) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetPosition(pos);
}

void CanBus::SetMitParam(const std::string& name, MitParam param) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitParam(param);
}

void CanBus::SetMitCmd(const std::string& name, float pos, float vel,
                       float effort, float kp, float kd) {
  auto* actr = GetActuator(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitCmd(pos, vel, effort, kp, kd);
}

}  // namespace xyber