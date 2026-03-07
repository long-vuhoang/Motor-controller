/*
 * @brief RobstrideMotor – Actuator implementation for Robstride 00 and 02.
 *
 * Protocol overview (Robstride CAN EFF)
 * ──────────────────────────────────────
 * All frames use 29-bit EXTENDED IDs (CAN_EFF_FLAG set on every TX frame).
 *
 * TX CAN ID layout (29-bit, big-endian field description):
 *
 *   bits [31:24] = communication_type   (8-bit command code)
 *   bits [23:8]  = extra_data           (for MIT: torque u16; else master_id=0xFF)
 *   bits [7:0]   = motor_id             (1-byte target motor ID, 1–3 on our buses)
 *   bit  [31]    = CAN_EFF_FLAG         (always set; marks extended frame)
 *
 * RX feedback CAN ID layout (type=0x02):
 *
 *   bits [31:24] = 0x02 (feedback comm_type)
 *   bits [23:8]  = extra_data:
 *                    [15:14] status_mode  (0=reset, 1=calib, 2=running/enabled)
 *                    [13]    uncalibrated
 *                    [12]    hall encoder fault
 *                    [11]    magnetic encoder fault
 *                    [10]    over-temperature
 *                    [9]     over-current
 *                    [8]     under-voltage
 *                    [7:0]   device_id (= motor_id in response)
 *   bits [7:0]   = master_id (host, 0x00 in feedback)
 *
 * RX data payload (8 bytes, big-endian u16 pairs):
 *   data[0:1] = position  u16,  range ±pos_max (rad)
 *   data[2:3] = velocity  u16,  range ±vel_max (rad/s)
 *   data[4:5] = torque    u16,  range ±toq_max (Nm)
 *   data[6:7] = temperature u16, scale × 0.1 °C
 *
 * MIT TX data payload (8 bytes):
 *   data[0:1] = position u16
 *   data[2:3] = velocity u16
 *   data[4:5] = kp       u16,  range [0, kp_max]
 *   data[6:7] = kd       u16,  range [0, kd_max]
 *   (torque feed-forward is packed into CAN ID bits [23:8])
 *
 * Set-parameter TX data payload (8 bytes):
 *   data[0]   = index low byte
 *   data[1]   = index high byte
 *   data[2:3] = 0x00
 *   data[4:7] = float value (little-endian) or uint8 in data[4]
 *
 * Reference: robstride-robstride_ros_sample / motor_cfg.cpp
 */

#pragma once

#include <cstdint>
#include <string>

#include "actuator_base.h"
#include "common_type.h"

namespace xyber {

class RobstrideMotor final : public Actuator {
 public:
  /* ── Communication types ──────────────────────────────────────────── */
  static constexpr uint8_t COMM_MIT_CONTROL  = 0x01; ///< Motion control (MIT)
  static constexpr uint8_t COMM_FEEDBACK     = 0x02; ///< Motor state feedback (RX)
  static constexpr uint8_t COMM_ENABLE       = 0x03; ///< Enable motor
  static constexpr uint8_t COMM_STOP         = 0x04; ///< Stop / disable motor
  static constexpr uint8_t COMM_SET_ZERO     = 0x06; ///< Set current position as zero
  static constexpr uint8_t COMM_SET_CAN_ID   = 0x07; ///< Change motor CAN ID
  static constexpr uint8_t COMM_GET_PARAM    = 0x11; ///< Read single parameter
  static constexpr uint8_t COMM_SET_PARAM    = 0x12; ///< Write single parameter

  /* ── Parameter register indices (motor_cfg.cpp / RobStride protocol) ─ */
  static constexpr uint16_t PARAM_RUN_MODE   = 0x7005; ///< Run mode (0–5)
  static constexpr uint16_t PARAM_IQ_REF     = 0x7006; ///< Current setpoint (A)
  static constexpr uint16_t PARAM_SPD_REF    = 0x700A; ///< Velocity setpoint (rad/s)
  static constexpr uint16_t PARAM_LOC_REF    = 0x7016; ///< Position setpoint (rad)
  static constexpr uint16_t PARAM_LIMIT_SPD  = 0x7017; ///< CSP speed limit
  static constexpr uint16_t PARAM_LIMIT_SPD2 = 0x7018; ///< Velocity mode speed limit
  static constexpr uint16_t PARAM_SPD_PP     = 0x7024; ///< PosPP speed
  static constexpr uint16_t PARAM_ACC_PP     = 0x7025; ///< PosPP acceleration
  static constexpr uint16_t PARAM_LIMIT_CUR  = 0x7026; ///< Velocity mode current limit

  /* ── Run mode codes written to PARAM_RUN_MODE ─────────────────────── */
  static constexpr uint8_t RUN_MIT      = 0; ///< Motion control (MIT)
  static constexpr uint8_t RUN_POS_PP   = 1; ///< Position mode – PP profile
  static constexpr uint8_t RUN_VEL      = 2; ///< Velocity mode
  static constexpr uint8_t RUN_CURRENT  = 3; ///< Current / torque mode
  static constexpr uint8_t RUN_SET_ZERO = 4; ///< Set-zero helper mode
  static constexpr uint8_t RUN_POS_CSP  = 5; ///< Position mode – CSP

  static constexpr uint8_t MASTER_ID = 0xFF; ///< Host (master) CAN ID

  /* ── Constructor ──────────────────────────────────────────────────── */
  /**
   * @param type     ActuatorType::Robstride_00 or Robstride_02.
   * @param name     Unique human-readable name for logging / lookup.
   * @param can_id   CAN node id on this bus: 1, 2, or 3.
   */
  RobstrideMotor(ActuatorType type, const std::string& name, uint8_t can_id);

  /* ── Actuator identity ────────────────────────────────────────────── */
  uint8_t            GetId()   const override { return can_id_; }
  const std::string& GetName() const override { return name_; }
  ActuatorType       GetType() const override { return type_; }

  /* ── Buffer registration ──────────────────────────────────────────── */
  void SetDataFiled(uint8_t* send_base, uint8_t* recv_base) override;

  /* ── One-shot commands (written by SendOnce, fired exactly once) ───── */
  void RequestState(ActuatorState state) override; ///< COMM_ENABLE or COMM_STOP
  void ClearError()                      override; ///< COMM_STOP with clear_error byte
  void SetHomingPosition()               override; ///< COMM_SET_ZERO
  void SaveConfig()                      override; ///< No-op (no flash-save in Robstride protocol)
  void SetMode(ActuatorMode mode)        override; ///< Write PARAM_RUN_MODE, track internally

  /* ── Periodic command writers (write to send_slot_ each cycle) ─────── */
  void SetMitParam(MitParam param)                                  override;
  void SetMitCmd(float pos, float vel,
                 float effort, float kp, float kd)                 override;
  void SetEffort(float cur)                                         override;
  void SetVelocity(float vel)                                       override;
  void SetPosition(float pos)                                       override;

  /* ── State readers (lazy-decode from recv_slot_ feedback) ──────────── */
  ActuatorState GetPowerState() const override;
  ActuatorMode  GetMode()       const override;
  float         GetTempure()    const override;
  std::string   GetErrorString() const override;
  float         GetEffort()     const override;
  float         GetVelocity()   const override;
  float         GetPosition()   const override;

 private:
  /* ── Motor limits per type ─────────────────────────────────────────── */
  struct Limits {
    float position; ///< ±rad
    float velocity; ///< ±rad/s
    float torque;   ///< ±Nm
    float kp;       ///< [0, kp_max]
    float kd;       ///< [0, kd_max]
  };
  const Limits& GetLimits() const;

  /* ── Quantisation helpers ──────────────────────────────────────────── */
  /**
   * @brief Map float in [x_min, x_max] → uint16 in [0, 65535].
   * Identical to motor_cfg.cpp float_to_uint(..., bits=16).
   */
  static uint16_t FloatToUint16(float x, float x_min, float x_max);

  /** Inverse of FloatToUint16. */
  static float Uint16ToFloat(uint16_t x, float x_min, float x_max);

  /* ── Low-level frame writers ───────────────────────────────────────── */
  /**
   * @brief Write a full EFF CanFrame into send_slot_.
   * @param comm_type   Communication type byte (COMM_*).
   * @param extra       bits[23:8] of the TX CAN ID (e.g. torque u16 or master_id).
   * @param data        8-byte payload; nullptr → all zeros.
   */
  void WriteFrame(uint8_t comm_type, uint16_t extra,
                  const uint8_t* data = nullptr);

  /**
   * @brief Write a COMM_SET_PARAM frame for a float parameter.
   * Sets data[0:1]=index (LE), data[2:3]=0, data[4:7]=value (LE float).
   */
  void WriteSetParamFloat(uint16_t index, float value);

  /**
   * @brief Write a COMM_SET_PARAM frame for an integer (mode) parameter.
   * Sets data[0:1]=index (LE), data[2:3]=0, data[4]=value, data[5:7]=0.
   */
  void WriteSetParamInt(uint16_t index, uint8_t value);

  /* ── Feedback decoder ──────────────────────────────────────────────── */
  /**
   * @brief Parse recv_slot_ into cached state fields.
   * Called lazily from every Get*() under recv_mtx_.
   * Updates: position_, velocity_, torque_, temperature_, power_state_.
   */
  void ParseFeedback() const;

  /* ── Members ───────────────────────────────────────────────────────── */
  ActuatorType type_;
  std::string  name_;
  uint8_t      can_id_;

  /// Pointer into CanBus::send_buf_ at slot (can_id-1) * sizeof(CanFrame).
  CanFrame* send_slot_ = nullptr;
  /// Pointer into CanBus::recv_buf_ at slot (can_id-1) * sizeof(CanFrame).
  CanFrame* recv_slot_ = nullptr;

  /// MIT parameters (user can override via SetMitParam()).
  MitParam mit_param_;

  /// Tracked internally; updated by SetMode() and ParseFeedback().
  mutable ActuatorMode  current_mode_  = ActuatorMode::MODE_MIT;
  mutable ActuatorState power_state_   = ActuatorState::STATE_DISABLE;

  /// Cached decoded feedback values; updated by ParseFeedback().
  mutable float position_    = 0.0f; ///< rad
  mutable float velocity_    = 0.0f; ///< rad/s
  mutable float torque_      = 0.0f; ///< Nm
  mutable float temperature_ = 0.0f; ///< °C

  /// Error flags decoded from last feedback extra_data.
  mutable uint8_t error_flags_ = 0;
};

}  // namespace xyber