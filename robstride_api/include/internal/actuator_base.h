/*
 * @brief Actuator – abstract base class for all motor types.
 *
 * Slot model (Robstride EFF protocol)
 * ─────────────────────────────────────
 * Each actuator owns two 12-byte slots inside CanBus buffers:
 *
 *   send_slot_ → CanFrame*  (can_id + 8 data bytes the actuator writes TX cmds into)
 *   recv_slot_ → CanFrame*  (can_id + 8 data bytes the RX thread writes feedback into)
 *
 * Slot layout in CanBus::send_buf_ / recv_buf_:
 *   Motor 1: bytes [  0.. 11 ]   (sizeof(CanFrame) * (id-1), id=1 → offset 0 )
 *   Motor 2: bytes [ 12.. 23 ]   (sizeof(CanFrame) * (id-1), id=2 → offset 12)
 *   Motor 3: bytes [ 24.. 35 ]   (sizeof(CanFrame) * (id-1), id=3 → offset 24)
 *
 * Call SetDataFiled() once after constructing the actuator (done by CanBus::RegisterActuator).
 *
 * Thread-safety
 * ─────────────
 * All Set*() / RequestState() methods write to send_slot_ — always called under send_mtx_.
 * All Get*() methods read from recv_slot_ or cached fields — always called under recv_mtx_.
 * No internal locking; CanBus owns the mutexes.
 */

#pragma once

#include <cstdint>
#include <string>

#include "common_type.h"

namespace xyber {

class Actuator {
 public:
  virtual ~Actuator() = default;

  /* ── Identity ─────────────────────────────────────────────────────── */
  virtual uint8_t           GetId()   const = 0;
  virtual const std::string& GetName() const = 0;
  virtual ActuatorType       GetType() const = 0;

  /* ── Buffer registration (called once by CanBus::RegisterActuator) ── */
  /**
   * @brief Point this actuator at its 12-byte CanFrame slots inside the
   *        CanBus send/recv buffers.
   *
   * @param send_base  send_buf_.data() — actuator adds (id-1)*sizeof(CanFrame)
   * @param recv_base  recv_buf_.data() — same offset formula
   */
  virtual void SetDataFiled(uint8_t* send_base, uint8_t* recv_base) = 0;

  /* ── One-shot command writers (called by CanBus via SendOnce) ──────── */
  virtual void RequestState(ActuatorState state) = 0;  ///< Enable / Disable
  virtual void ClearError()                      = 0;
  virtual void SetHomingPosition()               = 0;
  virtual void SaveConfig()                      = 0;
  virtual void SetMode(ActuatorMode mode)        = 0;

  /* ── Periodic command writers (called under send_mtx_) ────────────── */
  virtual void SetMitParam(MitParam param)                                  = 0;
  virtual void SetMitCmd(float pos, float vel,
                         float effort, float kp, float kd)                 = 0;
  virtual void SetEffort(float cur)                                         = 0;
  virtual void SetVelocity(float vel)                                       = 0;
  virtual void SetPosition(float pos)                                       = 0;

  /* ── State readers (called under recv_mtx_) ───────────────────────── */
  virtual ActuatorState GetPowerState() const = 0;
  virtual ActuatorMode  GetMode()       const = 0;
  virtual float         GetTempure()    const = 0;  ///< °C
  virtual std::string   GetErrorString() const = 0;
  virtual float         GetEffort()     const = 0;  ///< Nm
  virtual float         GetVelocity()   const = 0;  ///< rad/s
  virtual float         GetPosition()   const = 0;  ///< rad
};

}  // namespace xyber