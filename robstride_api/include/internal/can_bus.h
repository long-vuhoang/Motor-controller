/*
 * @brief CanBus – one SocketCAN interface, max 3 Robstride actuators.
 *
 * Protocol: CAN Extended Frame (EFF, 29-bit IDs).
 *
 * Robstride motors use CAN_EFF_FLAG on every TX frame.
 * The RX feedback frame carries the motor_id at bits [15:8] of the 29-bit ID,
 * NOT at bits [7:0] like standard 11-bit drivers.
 *
 * Slot model
 * ──────────
 * Each actuator owns a 12-byte CanFrame slot in send_buf_ / recv_buf_:
 *
 *   sizeof(CanFrame) = 4 (can_id) + 8 (data) = 12 bytes  (#pragma pack(1))
 *
 *   Motor id 1: offset  0..11  in buf
 *   Motor id 2: offset 12..23
 *   Motor id 3: offset 24..35
 *
 * The actuator writes the complete CanFrame (including can_id with EFF flag)
 * into its send slot.  TransmitAll() reads the slot and fires one frame per motor.
 * The per-actuator RX callback writes the received frame into the recv slot.
 *
 * SocketCAN key extractor
 * ───────────────────────
 * For Robstride feedback frames (comm_type=0x02), the motor_id occupies
 * bits [15:8] of the 29-bit ID:
 *
 *   [31:24]=0x02  [23:8]=extra_data  [15:8]=motor_id  [7:0]=host_id
 *
 * Key extractor: (frame.can_id >> 8) & 0xFF  → motor_id
 *
 * Thread-safety contract
 * ──────────────────────
 *   send_buf_  : written by Set*() / SendOnce() (user thread) under send_mtx_;
 *                read    by TransmitAll() (control-loop thread) under send_mtx_.
 *   recv_buf_  : written by per-actuator RX lambda (SocketCAN RX thread)
 *                under recv_mtx_;
 *                read    by Get*() callers (user thread) under recv_mtx_.
 *
 * Command routing
 * ───────────────
 *  PERIODIC  (SetMitCmd, SetEffort/Vel/Pos):
 *    Actuator writes full CanFrame to send_buf_ slot.
 *    TransmitAll() copies slot → can_frame and transmits every cycle.
 *
 *  ONE-SHOT  (Enable, Disable, ClearError, SetMode, SetHoming, SaveConfig):
 *    SendOnce(): write → copy → zero slot → transmit once.
 *    TransmitAll() NEVER re-broadcasts the one-shot command.
 */

#pragma once

#include <array>
#include <functional>
#include <linux/can.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "actuator_base.h"
#include "common_type.h"
#include "internal/common_utils.h"
#include "socket_can.hpp"

namespace xyber {

inline constexpr uint8_t MAX_ACTUATORS_PER_BUS = 4;

/*
 * Compile-time layout check.
 * Each buffer slot must be exactly sizeof(CanFrame) = 12 bytes.
 * Update common_utils.h: #define ACTUATOR_FRAME_SIZE sizeof(CanFrame)
 */
static_assert(sizeof(CanFrame) == 12,
              "CanFrame must be exactly 12 bytes (#pragma pack(1)).");

class CanBus {
 public:
  explicit CanBus(const std::string& interface);
  ~CanBus();

  CanBus(const CanBus&)            = delete;
  CanBus& operator=(const CanBus&) = delete;

  /* ── Registration ─────────────────────────────────────────────────── */
  bool RegisterActuator(Actuator* actr);
  Actuator*          GetActuator(const std::string& name);
  bool               HasActuator(const std::string& name) const;
  const std::string& GetInterface() const { return interface_; }

  /* ── Control-loop interface ────────────────────────────────────────── */
  void TransmitAll();

  /* ── Actuator API (all thread-safe) ───────────────────────────────── */
  bool EnableAll();
  bool DisableAll();
  bool EnableActuator(const std::string& name);
  bool DisableActuator(const std::string& name);

  void          ClearError(const std::string& name);
  void          SetHomingPosition(const std::string& name);
  void          SaveConfig(const std::string& name);

  float         GetTempure(const std::string& name);
  std::string   GetErrorString(const std::string& name);
  ActuatorState GetPowerState(const std::string& name);

  bool          SetMode(const std::string& name, ActuatorMode mode);
  ActuatorMode  GetMode(const std::string& name);

  void  SetEffort(const std::string& name, float cur);
  float GetEffort(const std::string& name);
  void  SetVelocity(const std::string& name, float vel);
  float GetVelocity(const std::string& name);
  void  SetPosition(const std::string& name, float pos);
  float GetPosition(const std::string& name);

  void SetMitParam(const std::string& name, MitParam param);
  void SetMitCmd(const std::string& name, float pos, float vel,
                 float effort, float kp, float kd);

 private:
  void SendOnce(Actuator* actr, const std::function<void()>& cmd_fn);

  std::string                interface_;
  std::shared_ptr<SocketCAN> socket_can_;

  /*
   * 3 × 12-byte CanFrame slots = 36 bytes per buffer.
   * Actuator pointers directly into these arrays via SetDataFiled().
   */
  alignas(8) std::array<uint8_t, MAX_ACTUATORS_PER_BUS * sizeof(CanFrame)> send_buf_{};
  alignas(8) std::array<uint8_t, MAX_ACTUATORS_PER_BUS * sizeof(CanFrame)> recv_buf_{};

  std::unordered_map<std::string, Actuator*> actuator_by_name_;
  std::unordered_map<uint8_t,     Actuator*> actuator_by_id_;

  mutable std::mutex send_mtx_;
  mutable std::mutex recv_mtx_;
};

}  // namespace xyber