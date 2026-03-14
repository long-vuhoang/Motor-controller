/*
 * @file  stats.h
 * @brief Online statistics + HDR histogram. Range 0–20 ms (1µs buckets).
 */
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include <time.h>

namespace bench {

// ── OnlineStats ───────────────────────────────────────────────────────────────

struct OnlineStats {
  void Reset() { n=0; mean=0; M2=0; min_ns=INT64_MAX; max_ns=0; }

  void Add(int64_t ns) {
    if (ns <= 0) return;
    ++n;
    double d  = (double)ns - mean;
    mean     += d / (double)n;
    M2       += d * ((double)ns - mean);
    if (ns < min_ns) min_ns = ns;
    if (ns > max_ns) max_ns = ns;
  }

  double MeanUs()   const { return mean * 1e-3; }
  double StddevUs() const { return n<2 ? 0.0 : std::sqrt(M2/(double)(n-1))*1e-3; }
  double MinUs()    const { return min_ns==INT64_MAX ? 0.0 : min_ns*1e-3; }
  double MaxUs()    const { return max_ns * 1e-3; }

  uint64_t n      = 0;
  double   mean   = 0.0;
  double   M2     = 0.0;
  int64_t  min_ns = INT64_MAX;
  int64_t  max_ns = 0;
};

// ── Histogram: 1µs buckets, 0–20ms ──────────────────────────────────────────

template <int64_t kBucketNs = 1000, size_t kMaxBuckets = 20000>
struct Histogram {
  static constexpr size_t  kBucketCount = kMaxBuckets;
  static constexpr int64_t kBucketWidth = kBucketNs;
  static constexpr int64_t kRangeNs     = (int64_t)kMaxBuckets * kBucketNs;

  void Reset() {
    for (auto& c : counts) c.store(0, std::memory_order_relaxed);
    overflow.store(0, std::memory_order_relaxed);
    total.store(0, std::memory_order_relaxed);
  }

  void Add(int64_t ns) {
    total.fetch_add(1, std::memory_order_relaxed);
    if (ns < 0) { overflow.fetch_add(1, std::memory_order_relaxed); return; }
    size_t idx = (size_t)(ns / kBucketNs);
    if (idx >= kMaxBuckets) overflow.fetch_add(1, std::memory_order_relaxed);
    else                    counts[idx].fetch_add(1, std::memory_order_relaxed);
  }

  double Percentile(double p) const {
    uint64_t n = total.load(std::memory_order_relaxed);
    if (!n) return 0.0;
    uint64_t target = (uint64_t)(p/100.0*(double)n);
    uint64_t cum = 0;
    for (size_t i = 0; i < kMaxBuckets; ++i) {
      cum += counts[i].load(std::memory_order_relaxed);
      if (cum > target) return (double)i * kBucketNs * 1e-3;
    }
    return (double)kRangeNs * 1e-3;
  }

  double P50Us()  const { return Percentile(50.0); }
  double P90Us()  const { return Percentile(90.0); }
  double P99Us()  const { return Percentile(99.0); }
  double P999Us() const { return Percentile(99.9); }

  std::array<std::atomic<uint32_t>, kMaxBuckets> counts{};
  std::atomic<uint64_t> overflow{0};
  std::atomic<uint64_t> total{0};
};

// ── LatencySample ─────────────────────────────────────────────────────────────

struct LatencySample {
  std::string name;
  OnlineStats stats;
  Histogram<> histogram;   // 1µs bucket, 0–20ms

  void Reset() { stats.Reset(); histogram.Reset(); }
  void Add(int64_t ns) { stats.Add(ns); histogram.Add(ns); }
};

// ── Clock helpers ─────────────────────────────────────────────────────────────

[[nodiscard]] inline int64_t NowNs() {
  struct timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1'000'000'000LL + ts.tv_nsec;
}

inline void SleepUntil(int64_t deadline_ns) {
  struct timespec ts{};
  ts.tv_sec  = deadline_ns / 1'000'000'000LL;
  ts.tv_nsec = deadline_ns % 1'000'000'000LL;
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

}  // namespace bench