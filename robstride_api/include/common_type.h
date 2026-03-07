/*
 * @Author: Long Vu
 * @Date:   2026-01-11 10:30:00
 * @Description: Base config parameters for actuator control
 * @Version: 1.0
 * @License: MIT
 * @Copyright: (c) 2026 Long Vu
 */

#pragma once

// CPP
#include <cmath>
#include <cstdint>
#include <string>

namespace xyber {

#pragma pack(1)
// Communication buffers
struct CanFrame {
  uint32_t can_id;  // Motor ID control byte
  uint8_t data[8];  // CAN data
};

#pragma pack()

enum class CtrlChannel : uint8_t {
  CH1 = 1,
  CH2 = 2,
  CH3 = 3,
  CH4 = 4,
};

enum class ActuatorType {
  Robstride_00,
  Robstride_02,
  Servo,
  UNKNOWN,
};

static ActuatorType StringToType(std::string type) {
  if (type == "Robstride_00") return ActuatorType::Robstride_00;
  if (type == "Robstride_02") return ActuatorType::Robstride_02;
  if (type == "Servo") return ActuatorType::Servo;
  return ActuatorType::UNKNOWN;
}

enum ActuatorState : uint8_t {
  STATE_DISABLE = 0,
  STATE_ENABLE = 1,
  STATE_CALIBRATION = 2,
};

enum ActuatorMode : uint8_t {
  MODE_MIT = 0,
  MODE_POSITION = 1,
  MODE_VELOCITY = 2,
  MODE_CURRENT = 3,
  MODE_ZERO = 4,
};

/**
 * @description: Mit parameters
 */
struct MitParam {
  float pos_min = 0.0;
  float pos_max = 0.0;
  float vel_min = 0.0;
  float vel_max = 0.0;
  float toq_min = 0.0;
  float toq_max = 0.0;
  float kp_min = 0.0;
  float kp_max = 0.0;
  float kd_min = 0.0;
  float kd_max = 0.0;
};

#define ROBSTRIDE_00_MIT_MODE_DEFAULT_PARAM \
  {                                         \
      .pos_min = -4.0f * M_PI,              \
      .pos_max = 4.0f * M_PI,               \
      .vel_min = -44.0f,                    \
      .vel_max = 44.0f,                     \
      .toq_min = -14.0f,                    \
      .toq_max = 14.0f,                     \
      .kp_min = 0.0f,                       \
      .kp_max = 500.0f,                     \
      .kd_min = 0.0f,                       \
      .kd_max = 5.0f,                       \
  }

#define ROBSTRIDE_02_MIT_MODE_DEFAULT_PARAM \
  {                                         \
      .pos_min = -4.0f * M_PI,              \
      .pos_max = 4.0f * M_PI,               \
      .vel_min = -44.0f,                    \
      .vel_max = 44.0f,                     \
      .toq_min = -17.0f,                    \
      .toq_max = 17.0f,                     \
      .kp_min = 0.0f,                       \
      .kp_max = 500.0f,                     \
      .kd_min = 0.0f,                       \
      .kd_max = 5.0f,                       \
  }

}  // namespace xyber