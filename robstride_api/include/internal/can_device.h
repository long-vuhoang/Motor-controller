/*
 * @brief CanDevice – one physical hardware unit with up to 4 CAN buses.
 *
 * Replaces the Dcu + EthercatManager pair.
 *
 * Responsibilities
 * ────────────────
 *  • Creates and owns MAX_CAN_BUSES CanBus instances (one per interface).
 *  • Provides a single realtime control-loop thread that calls
 *    CanBus::TransmitAll() on every bus each cycle.
 *  • Maintains a fast actuator-name → owning-CanBus map for O(1) dispatch.
 *  • Exposes the same actuator API as the original Dcu class.
 *
 * Hardware topology
 * ─────────────────
 *   1 CanDevice  =  4 CAN buses  =  up to 12 actuators total
 *   1 CanBus     =  max 3 actuators  (CAN ids 1, 2, 3)
 *
 * Thread model
 * ────────────
 *  SocketCAN already runs one RX thread and one TX thread per interface,
 *  both at SCHED_FIFO priority 80.  CanDevice adds one control-loop thread
 *  (configurable RT priority) that fires TransmitAll() at the chosen cycle rate.
 *
 *  ┌────────────────────────────────────────────────────┐
 *  │ CanDevice control-loop thread (rt, configurable)   │
 *  │   every cycle_ns → for each bus → TransmitAll()   │
 *  └────────────────────────────────────────────────────┘
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │ SocketCAN per-bus threads (rt priority 80, per interface)   │
 *  │   can_rx → OnCanFrame() → recv_buf_                         │
 *  │   can_tx ← queue fed by TransmitAll()                       │
 *  └─────────────────────────────────────────────────────────────┘
 */

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "common_type.h"
#include "can_bus.h"

namespace xyber {

inline constexpr uint8_t MAX_CAN_BUSES = 4;

class CanDevice {
 public:
  /**
   * @param name        Unique device name used in log messages.
   * @param interfaces  Exactly MAX_CAN_BUSES CAN interface names
   *                    (e.g. {"can0","can1","can2","can3"}).
   *                    Pass empty strings for unused buses.
   */
  explicit CanDevice(std::string name,
                     std::array<std::string, MAX_CAN_BUSES> interfaces);
  ~CanDevice();

  CanDevice(const CanDevice&) = delete;
  CanDevice& operator=(const CanDevice&) = delete;

  const std::string& GetName() const { return name_; }
  bool IsRunning() const { return is_running_.load(); }

  /* ── Actuator registration (call BEFORE Start()) ──────────────────────── */

  /**
   * @brief Attach an actuator to one CAN bus of this device.
   * @param bus_idx  Bus index 0 … MAX_CAN_BUSES-1
   * @param type     ActuatorType (POWER_FLOW_R86, R52, L28, OMNI_PICKER)
   * @param name     Globally unique actuator name
   * @param can_id   CAN id on the bus: 1, 2, or 3
   * @return true on success
   */
  bool AttachActuator(uint8_t bus_idx, ActuatorType type,
                      const std::string& name, uint8_t can_id);

  /* ── Life-cycle ───────────────────────────────────────────────────────── */

  /**
   * @brief Start the cyclic control-loop thread.
   * @param cycle_ns    Loop period in nanoseconds (default 1 ms = 1 000 000 ns)
   * @param rt_priority SCHED_FIFO priority [0,99]; -1 = normal scheduling
   * @param bind_cpu    CPU core affinity; -1 = no affinity
   */
  bool Start(uint64_t cycle_ns = 1'000'000, int rt_priority = -1, int bind_cpu = -1);
  void Stop();

  /* ── Actuator API (all thread-safe, delegate to owning CanBus) ────────── */

  bool EnableAllActuator();
  bool EnableActuator(const std::string& name);
  bool DisableAllActuator();
  bool DisableActuator(const std::string& name);

  void ClearError(const std::string& name);
  void SetHomingPosition(const std::string& name);
  void SaveConfig(const std::string& name);

  float         GetTempure(const std::string& name);
  std::string   GetErrorString(const std::string& name);
  ActuatorState GetPowerState(const std::string& name);

  bool         SetMode(const std::string& name, ActuatorMode mode);
  ActuatorMode GetMode(const std::string& name);

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
  /** Realtime loop: fires TransmitAll() on every active bus each cycle. */
  void ControlLoop(uint64_t cycle_ns, int rt_priority, int bind_cpu);

  /** Returns the CanBus that owns the named actuator, or nullptr. */
  CanBus* FindBus(const std::string& actuator_name);

  std::string name_;

  /* One CanBus per physical CAN port; unique_ptr owns lifetime. */
  std::array<std::unique_ptr<CanBus>, MAX_CAN_BUSES> buses_;

  /* actuator name → owning bus (non-owning pointer, populated at attach time). */
  std::unordered_map<std::string, CanBus*> actuator_bus_map_;

  std::thread       control_loop_thread_;
  std::atomic_bool  is_running_{false};
};

}  // namespace xyber