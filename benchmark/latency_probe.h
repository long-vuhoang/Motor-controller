/*
 * @file  latency_probe.h
 * @brief ControlLoopProbe + MutexProbe.
 *        RTL is now measured by CanSniffer (can_sniffer.h), not here.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "stats.h"

namespace bench {

// ── ControlLoopProbe ──────────────────────────────────────────────────────────

class ControlLoopProbe {
 public:
  ControlLoopProbe(const std::string& name, uint64_t target_cycle_ns)
      : name_(name), target_ns_((int64_t)target_cycle_ns) {
    jitter_.name = name + "_jitter";
    period_.name = name + "_period";
  }

  void Tick() {
    int64_t now = NowNs();
    if (last_ != 0) {
      int64_t actual = now - last_;
      period_.Add(actual);
      int64_t j = actual > target_ns_ ? actual - target_ns_
                                      : target_ns_ - actual;
      jitter_.Add(j);
    }
    last_ = now;
  }

  const LatencySample& JitterSample() const { return jitter_; }
  const LatencySample& PeriodSample() const { return period_; }

 private:
  std::string   name_;
  int64_t       target_ns_;
  int64_t       last_ = 0;
  LatencySample jitter_;
  LatencySample period_;
};

// ── MutexProbe ────────────────────────────────────────────────────────────────

class MutexProbe {
 public:
  explicit MutexProbe(const std::string& name) {
    wait_.name = name + "_lock_wait";
    held_.name = name + "_lock_held";
  }

  [[nodiscard]] int64_t BeginWait() const { return NowNs(); }

  void Acquired(int64_t wait_start) {
    wait_.Add(NowNs() - wait_start);
    held_start_ = NowNs();
  }
  void Released() {
    if (held_start_) { held_.Add(NowNs() - held_start_); held_start_ = 0; }
  }

  const LatencySample& WaitSample() const { return wait_; }
  const LatencySample& HeldSample() const { return held_; }

 private:
  LatencySample wait_, held_;
  int64_t held_start_ = 0;
};

}  // namespace bench