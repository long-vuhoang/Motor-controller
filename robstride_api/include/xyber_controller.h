/*
 * @brief XyberController – public singleton façade (SocketCAN build).
 *
 * API changes from the EtherCAT build
 * ─────────────────────────────────────
 *  • CreateDcu(name, ecat_id)            →  CreateDevice(name, {4 interfaces})
 *  • AttachActuator(dcu, CtrlChannel, …) →  AttachActuator(device, bus_idx, …)
 *  • Start(ifname, cycle_ns, enable_dc)  →  Start(cycle_ns)
 *
 * Everything else (Enable/Disable, Get/Set position/velocity/effort, MIT mode,
 * SetRealtime, Stop) is unchanged.
 *
 * IMU note
 * ────────
 * The original DCU IMU was an EtherCAT-specific peripheral.  Stub methods are
 * provided; a CAN-based IMU driver can be wired in by subclassing CanDevice.
 */

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include "common_type.h"

namespace xyber {

inline constexpr uint8_t CONTROLLER_MAX_CAN_BUSES = 4;

class XyberController {
 public:
  /* Singleton – non-copyable, non-assignable */
  XyberController(const XyberController&) = delete;
  void operator=(const XyberController&) = delete;
  virtual ~XyberController();

  /** @brief Get (or lazily create) the singleton instance. */
  static XyberController* GetInstance();

  /** @brief Return version string "major.minor.patch". */
  std::string GetVersion();

  /* ── Configuration (call before Start()) ─────────────────────────────── */

  /**
   * @brief Create a CanDevice backed by up to 4 SocketCAN interfaces.
   *
   * @param name        Unique device name.
   * @param interfaces  Array of 4 interface strings.  Pass "" for unused buses.
   *                    Example: {"can0", "can1", "can2", "can3"}
   */
  bool CreateDevice(const std::string& name,
                    std::array<std::string, CONTROLLER_MAX_CAN_BUSES> interfaces);

  /**
   * @brief Attach an actuator to one bus of a previously created device.
   *
   * @param device_name  Name passed to CreateDevice().
   * @param bus_idx      Bus index 0–3.
   * @param type         Actuator model (POWER_FLOW_R86, R52, L28, OMNI_PICKER).
   * @param actr_name    Globally unique actuator name for later lookup.
   * @param can_id       CAN node id on that bus: 1, 2, or 3.
   */
  bool AttachActuator(const std::string& device_name,
                      uint8_t            bus_idx,
                      ActuatorType       type,
                      const std::string& actr_name,
                      uint8_t            can_id);

  /**
   * @brief Configure the realtime control-loop thread.
   *
   * Must be called before Start().
   * @param rt_priority  SCHED_FIFO priority [0,99].  -1 = normal scheduler.
   * @param bind_cpu     CPU core to pin the thread.  -1 = no affinity.
   */
  bool SetRealtime(int rt_priority, int bind_cpu);

  /**
   * @brief Start all device control loops.
   * @param cycle_ns  TX period in nanoseconds (default 1 ms).
   */
  bool Start(uint64_t cycle_ns = 1'000'000);

  /** @brief Stop all device control loops and join threads. */
  void Stop();

 public: /* ── Actuator API ──────────────────────────────────────────────── */

  bool EnableAllActuator();
  bool EnableActuator(const std::string& name);
  bool DisableAllActuator();
  bool DisableActuator(const std::string& name);

  /**
   * @brief Read motor temperature.
   * @return Temperature in °C (implementation-defined units for OmniPicker).
   */
  float GetTempure(const std::string& name);

  ActuatorState GetPowerState(const std::string& name);
  ActuatorMode  GetMode(const std::string& name);

  /** @return Torque / current in Nm (PowerFlowR) or normalised 0–1 (others). */
  float GetEffort(const std::string& name);

  /** @return Angular velocity in rad/s. */
  float GetVelocity(const std::string& name);

  /** @return Position in rad. */
  float GetPosition(const std::string& name);

  /** @brief Override the MIT parameter set for an actuator. */
  void SetMitParam(const std::string& name, MitParam param);

  /**
   * @brief Send a MIT (model-based impedance-torque) command.
   *
   * For PowerFlowL / OmniPicker only pos and effort are used;
   * pass 0 for vel, kp, kd.
   *
   * @param pos     Target position (rad)
   * @param vel     Target velocity (rad/s)
   * @param effort  Feed-forward torque (Nm)
   * @param kp      Position gain
   * @param kd      Velocity / damping gain
   */
  void SetMitCmd(const std::string& name,
                 float pos, float vel, float effort, float kp, float kd);

 protected:
  XyberController();

 private:
  std::atomic_bool is_running_{false};
  int rt_priority_ = -1;
  int bind_cpu_    = -1;

  static XyberController* instance_;
};

using XyberControllerPtr = std::shared_ptr<XyberController>;

}  // namespace xyber