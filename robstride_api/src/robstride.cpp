/*
 * @brief RobstrideMotor implementation.
 *
 * See robstride_motor.h for full protocol documentation.
 */

#include "internal/robstride.h"

#include <cstring>
#include <linux/can.h>
#include <sstream>

namespace xyber {

// ── Constructor ──────────────────────────────────────────────────────────────

RobstrideMotor::RobstrideMotor(ActuatorType   type,
                               const std::string& name,
                               uint8_t            can_id)
    : type_(type), name_(name), can_id_(can_id) {
  /*
   * Initialise MIT parameters from compile-time defaults in common_type.h.
   * The user can override with SetMitParam() before starting the control loop.
   */
  if (type_ == ActuatorType::Robstride_00) {
    mit_param_ = MitParam ROBSTRIDE_00_MIT_MODE_DEFAULT_PARAM;
  } else {
    mit_param_ = MitParam ROBSTRIDE_02_MIT_MODE_DEFAULT_PARAM;
  }
}

// ── Buffer registration ──────────────────────────────────────────────────────

void RobstrideMotor::SetDataFiled(uint8_t* send_base, uint8_t* recv_base) {
  const size_t offset = sizeof(CanFrame) * static_cast<size_t>(can_id_ - 1);
  send_slot_ = reinterpret_cast<CanFrame*>(send_base + offset);
  recv_slot_ = reinterpret_cast<CanFrame*>(recv_base + offset);

  /* Zero-init both slots so TransmitAll() skips this motor until the first
   * periodic command is written (can_id == 0 → skip in TransmitAll). */
  std::memset(send_slot_, 0, sizeof(CanFrame));
  std::memset(recv_slot_, 0, sizeof(CanFrame));
}

// ── Motor limits per type ────────────────────────────────────────────────────

const RobstrideMotor::Limits& RobstrideMotor::GetLimits() const {
  /*
   * Values taken directly from common_type.h ROBSTRIDE_0x_MIT_MODE_DEFAULT_PARAM.
   * Robstride 00: torque ±14 Nm,  Robstride 02: torque ±17 Nm.
   * Velocity and position limits are identical for both models.
   */
  static const Limits kLimits00 = {
      .position = 4.0f * static_cast<float>(M_PI),
      .velocity = 44.0f,
      .torque   = 14.0f,
      .kp       = 500.0f,
      .kd       = 5.0f,
  };
  static const Limits kLimits02 = {
      .position = 4.0f * static_cast<float>(M_PI),
      .velocity = 44.0f,
      .torque   = 17.0f,
      .kp       = 500.0f,
      .kd       = 5.0f,
  };
  return (type_ == ActuatorType::Robstride_00) ? kLimits00 : kLimits02;
}

// ── Quantisation helpers ─────────────────────────────────────────────────────

uint16_t RobstrideMotor::FloatToUint16(float x, float x_min, float x_max) {
  /*
   * Identical to motor_cfg.cpp float_to_uint(x, x_min, x_max, bits=16).
   * Maps [x_min, x_max] → [0, 65535].
   */
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return static_cast<uint16_t>(((x - x_min) / (x_max - x_min)) * 65535.0f);
}

float RobstrideMotor::Uint16ToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (static_cast<float>(x) / 65535.0f) * (x_max - x_min);
}

// ── Low-level frame writers ──────────────────────────────────────────────────

void RobstrideMotor::WriteFrame(uint8_t        comm_type,
                                uint16_t       extra,
                                const uint8_t* data) {
  if (!send_slot_) return;

  /*
   * Robstride EFF TX CAN ID:
   *   bits [31:24] = comm_type
   *   bits [23:8]  = extra  (torque u16 for MIT; master_id=0xFF for others)
   *   bits [7:0]   = can_id_ (motor node id)
   *   CAN_EFF_FLAG  set always
   */
  send_slot_->can_id = (static_cast<uint32_t>(comm_type) << 24) |
                       (static_cast<uint32_t>(extra)      <<  8) |
                        static_cast<uint32_t>(can_id_)           |
                        CAN_EFF_FLAG;

  if (data) {
    std::memcpy(send_slot_->data, data, 8);
  } else {
    std::memset(send_slot_->data, 0, 8);
  }
}

void RobstrideMotor::WriteSetParamFloat(uint16_t index, float value) {
  uint8_t d[8] = {};
  d[0] = static_cast<uint8_t>(index & 0xFF);
  d[1] = static_cast<uint8_t>(index >> 8);
  /* d[2:3] = 0x00 (reserved) */
  std::memcpy(&d[4], &value, sizeof(float)); /* little-endian float */

  WriteFrame(COMM_SET_PARAM,
             static_cast<uint16_t>(MASTER_ID),   /* extra = master_id */
             d);
}

void RobstrideMotor::WriteSetParamInt(uint16_t index, uint8_t value) {
  uint8_t d[8] = {};
  d[0] = static_cast<uint8_t>(index & 0xFF);
  d[1] = static_cast<uint8_t>(index >> 8);
  d[4] = value;

  WriteFrame(COMM_SET_PARAM,
             static_cast<uint16_t>(MASTER_ID),
             d);
}

// ── One-shot command writers ─────────────────────────────────────────────────

void RobstrideMotor::RequestState(ActuatorState state) {
  if (state == ActuatorState::STATE_ENABLE) {
    /*
     * COMM_ENABLE frame: payload all zeros.
     * motor_cfg.cpp: enable_motor() → frame.data = memset(0, 8).
     */
    WriteFrame(COMM_ENABLE,
               static_cast<uint16_t>(MASTER_ID));
  } else {
    /*
     * COMM_STOP frame: data[0] = clear_error (0 = don't clear).
     * motor_cfg.cpp: Disenable_Motor(clear_error=0).
     */
    uint8_t d[8] = {};
    WriteFrame(COMM_STOP,
               static_cast<uint16_t>(MASTER_ID),
               d);
  }
}

void RobstrideMotor::ClearError() {
  /*
   * Same as Disenable_Motor(clear_error=1) in motor_cfg.cpp.
   * data[0] = 1 signals firmware to reset the fault register.
   */
  uint8_t d[8] = {};
  d[0] = 1;
  WriteFrame(COMM_STOP,
             static_cast<uint16_t>(MASTER_ID),
             d);
}

void RobstrideMotor::SetHomingPosition() {
  /*
   * COMM_SET_ZERO: Set current mechanical position as the zero reference.
   * motor_cfg.cpp: Set_ZeroPos() → frame.data[0]=1, rest 0.
   */
  uint8_t d[8] = {};
  d[0] = 1;
  WriteFrame(COMM_SET_ZERO,
             static_cast<uint16_t>(MASTER_ID),
             d);
}

void RobstrideMotor::SaveConfig() {
  /*
   * The Robstride protocol (as used in the reference sample) does not expose
   * an explicit flash-save command.  Configuration is automatically persisted
   * by the motor firmware.  This is a no-op: we zero the send slot so
   * CanBus::SendOnce() transmits nothing meaningful.
   */
  if (send_slot_) std::memset(send_slot_, 0, sizeof(CanFrame));
}

void RobstrideMotor::SetMode(ActuatorMode mode) {
  /*
   * Write PARAM_RUN_MODE via COMM_SET_PARAM.
   *
   * IMPORTANT: The reference firmware requires the motor to be stopped before
   * changing the run mode.  Callers should:
   *   1. DisableActuator(name)
   *   2. SetMode(name, mode)
   *   3. EnableActuator(name)
   *
   * ActuatorMode  → Robstride run_mode byte
   * ──────────────────────────────────────
   * MODE_MIT      → RUN_MIT     (0)
   * MODE_POSITION → RUN_POS_PP  (1)
   * MODE_VELOCITY → RUN_VEL     (2)
   * MODE_CURRENT  → RUN_CURRENT (3)
   * MODE_ZERO     → RUN_SET_ZERO(4)
   */
  static constexpr uint8_t kModeMap[] = {
      RUN_MIT,       // MODE_MIT      = 0
      RUN_POS_PP,    // MODE_POSITION = 1
      RUN_VEL,       // MODE_VELOCITY = 2
      RUN_CURRENT,   // MODE_CURRENT  = 3
      RUN_SET_ZERO,  // MODE_ZERO     = 4
  };

  const uint8_t idx = static_cast<uint8_t>(mode);
  if (idx >= sizeof(kModeMap)) return;

  WriteSetParamInt(PARAM_RUN_MODE, kModeMap[idx]);

  /* Track internally so GetMode() returns the new mode immediately. */
  current_mode_ = mode;
}

// ── Periodic command writers ─────────────────────────────────────────────────

void RobstrideMotor::SetMitParam(MitParam param) {
  mit_param_ = param;
}

void RobstrideMotor::SetMitCmd(float pos, float vel,
                               float effort, float kp, float kd) {
  /*
   * MIT frame layout (motor_cfg.cpp send_motion_command):
   *
   *   CAN ID bits[23:8] = torque u16  (range ± torque_max)
   *   data[0:1]         = position u16 (range ± pos_max)
   *   data[2:3]         = velocity u16 (range ± vel_max)
   *   data[4:5]         = kp       u16 (range [0, kp_max])
   *   data[6:7]         = kd       u16 (range [0, kd_max])
   *
   * All values quantised to 16 bits (FloatToUint16).
   */
  const Limits& lim = GetLimits();

  const uint16_t toq_u16 = FloatToUint16(effort, -lim.torque,   lim.torque);
  const uint16_t pos_u16 = FloatToUint16(pos,    -lim.position, lim.position);
  const uint16_t vel_u16 = FloatToUint16(vel,    -lim.velocity, lim.velocity);
  const uint16_t kp_u16  = FloatToUint16(kp,     0.0f,          lim.kp);
  const uint16_t kd_u16  = FloatToUint16(kd,     0.0f,          lim.kd);

  uint8_t d[8];
  d[0] = static_cast<uint8_t>(pos_u16 >> 8);
  d[1] = static_cast<uint8_t>(pos_u16 & 0xFF);
  d[2] = static_cast<uint8_t>(vel_u16 >> 8);
  d[3] = static_cast<uint8_t>(vel_u16 & 0xFF);
  d[4] = static_cast<uint8_t>(kp_u16  >> 8);
  d[5] = static_cast<uint8_t>(kp_u16  & 0xFF);
  d[6] = static_cast<uint8_t>(kd_u16  >> 8);
  d[7] = static_cast<uint8_t>(kd_u16  & 0xFF);

  WriteFrame(COMM_MIT_CONTROL, toq_u16, d);
}

void RobstrideMotor::SetVelocity(float vel) {
  /*
   * Velocity mode: write PARAM_SPD_REF (0x700A) via COMM_SET_PARAM.
   * motor_cfg.cpp send_velocity_mode_command → Set_RobStrite_Motor_parameter(0x700A, vel).
   */
  WriteSetParamFloat(PARAM_SPD_REF, vel);
}

void RobstrideMotor::SetPosition(float pos) {
  /*
   * Position mode (PP/CSP): write PARAM_LOC_REF (0x7016).
   * motor_cfg.cpp RobStrite_Motor_PosPP_control → Set_RobStrite_Motor_parameter(0x7016, angle).
   */
  WriteSetParamFloat(PARAM_LOC_REF, pos);
}

void RobstrideMotor::SetEffort(float cur) {
  /*
   * Current mode: write PARAM_IQ_REF (0x7006).
   * motor_cfg.cpp RobStrite_Motor_Current_control → Set_RobStrite_Motor_parameter(0x7006, iq).
   */
  WriteSetParamFloat(PARAM_IQ_REF, cur);
}

// ── Feedback decoder ─────────────────────────────────────────────────────────

void RobstrideMotor::ParseFeedback() const {
  if (!recv_slot_) return;

  const uint32_t raw_id = recv_slot_->can_id;

  /* Skip if no valid EFF feedback has arrived yet (slot is zero-init). */
  if (!(raw_id & CAN_EFF_FLAG)) return;

  const uint32_t id_val   = raw_id & CAN_EFF_MASK;
  const uint8_t  comm     = static_cast<uint8_t>((id_val >> 24) & 0xFF);

  if (comm != COMM_FEEDBACK) return;   /* only decode type-2 feedback */

  /*
   * Extra data field (bits[23:8]) encodes motor status:
   *   bits[15:14] status_mode   (0=reset, 1=calib, 2=running=enabled)
   *   bit[13]     uncalibrated
   *   bit[12]     hall encoder fault
   *   bit[11]     magnetic encoder fault
   *   bit[10]     over-temperature
   *   bit[9]      over-current
   *   bit[8]      under-voltage
   *   bits[7:0]   device_id (mirrors motor's own CAN id)
   */
  const uint16_t extra       = static_cast<uint16_t>((id_val >> 8) & 0xFFFF);
  const uint8_t  status_mode = static_cast<uint8_t>((extra >> 14) & 0x03);
  error_flags_ = static_cast<uint8_t>((extra >> 8) & 0x3F); /* bits[13:8] */

  power_state_ = (status_mode == 2)
                     ? ActuatorState::STATE_ENABLE
                     : ActuatorState::STATE_DISABLE;

  /*
   * Data payload: big-endian uint16 pairs.
   * motor_cfg.cpp receive_status_frame() Communication_Type_MotorRequest block.
   */
  const uint8_t* d = recv_slot_->data;
  const uint16_t pos_u16  = static_cast<uint16_t>((d[0] << 8) | d[1]);
  const uint16_t vel_u16  = static_cast<uint16_t>((d[2] << 8) | d[3]);
  const uint16_t toq_u16  = static_cast<uint16_t>((d[4] << 8) | d[5]);
  const uint16_t temp_u16 = static_cast<uint16_t>((d[6] << 8) | d[7]);

  const Limits& lim = GetLimits();
  position_    = Uint16ToFloat(pos_u16, -lim.position, lim.position);
  velocity_    = Uint16ToFloat(vel_u16, -lim.velocity, lim.velocity);
  torque_      = Uint16ToFloat(toq_u16, -lim.torque,   lim.torque);
  temperature_ = static_cast<float>(temp_u16) * 0.1f;
}

// ── State readers ────────────────────────────────────────────────────────────

ActuatorState RobstrideMotor::GetPowerState() const {
  ParseFeedback();
  return power_state_;
}

ActuatorMode RobstrideMotor::GetMode() const {
  return current_mode_;
}

float RobstrideMotor::GetTempure() const {
  ParseFeedback();
  return temperature_;
}

float RobstrideMotor::GetEffort() const {
  ParseFeedback();
  return torque_;
}

float RobstrideMotor::GetVelocity() const {
  ParseFeedback();
  return velocity_;
}

float RobstrideMotor::GetPosition() const {
  ParseFeedback();
  return position_;
}

std::string RobstrideMotor::GetErrorString() const {
  ParseFeedback();
  if (error_flags_ == 0) return "OK";

  std::ostringstream oss;
  if (error_flags_ & (1 << 5)) oss << "uncalibrated ";
  if (error_flags_ & (1 << 4)) oss << "hall_encoder_fault ";
  if (error_flags_ & (1 << 3)) oss << "magnetic_encoder_fault ";
  if (error_flags_ & (1 << 2)) oss << "over_temperature ";
  if (error_flags_ & (1 << 1)) oss << "over_current ";
  if (error_flags_ & (1 << 0)) oss << "under_voltage ";

  std::string result = oss.str();
  if (!result.empty() && result.back() == ' ') result.pop_back();
  return result;
}

}  // namespace xyber