/*
 * @file  report.h
 * @brief Terminal table + CSV + ASCII histogram.
 *
 * Thresholds (p99):
 *   RTL       PASS < 600 µs   WARN < 1200 µs   (CAN 1Mbps + Robstride firmware)
 *   Jitter    PASS <  50 µs   WARN <  100 µs   (5% of 1ms cycle)
 *   Period    PASS <  20 µs   WARN <   50 µs   (deviation from target, not absolute)
 *   Mutex     PASS <  10 µs   WARN <   50 µs
 */
#pragma once

#include <cstdio>
#include <fstream>
#include <inttypes.h>
#include <string>
#include <vector>

#include "stats.h"

namespace bench {

struct Thresholds { double pass_p99_us, warn_p99_us; };

// Realistic thresholds based on Robstride 1Mbps CAN
inline const Thresholds kRTLThresh    = {  600.0, 1200.0 };
inline const Thresholds kJitterThresh = {   50.0,  100.0 };
inline const Thresholds kPeriodThresh = {   20.0,   50.0 };  // deviation from target
inline const Thresholds kMutexThresh  = {   10.0,   50.0 };

namespace col {
  constexpr const char* R = "\033[0m";
  constexpr const char* G = "\033[32m";
  constexpr const char* Y = "\033[33m";
  constexpr const char* E = "\033[31m";   // Error/red
  constexpr const char* B = "\033[1m";
  constexpr const char* C = "\033[36m";
}

inline const char* Verdict(double p99, const Thresholds& t) {
  return (p99 < t.pass_p99_us) ? "PASS" : (p99 < t.warn_p99_us) ? "WARN" : "FAIL";
}
inline const char* VC(const char* v) {
  return (v[0]=='P') ? col::G : (v[0]=='W') ? col::Y : col::E;
}

struct Row {
  std::string name;
  uint64_t n=0;
  double mean=0,std=0,min=0,p50=0,p90=0,p99=0,p999=0,max=0;
  const char* verdict="";
  uint64_t overflow=0;
};

class Report {
 public:
  explicit Report(const std::string& title) : title_(title) {}

  void Add(const LatencySample& s, const Thresholds& t) {
    double p99 = s.histogram.P99Us();
    rows_.push_back({
      s.name, s.stats.n,
      s.stats.MeanUs(), s.stats.StddevUs(), s.stats.MinUs(),
      s.histogram.P50Us(), s.histogram.P90Us(), p99,
      s.histogram.P999Us(), s.stats.MaxUs(),
      Verdict(p99, t),
      s.histogram.overflow.load()
    });
  }

  void Sep() { rows_.push_back({"---"}); }

  void Print() const {
    const char* line =
      "────────────────────────────────────────────────────────────"
      "─────────────────────────────────────────────────────\n";
    printf("\n%s%s%s\n", col::B, title_.c_str(), col::R);
    printf("%s", line);
    printf("%s%-36s %7s %7s %7s %7s %7s %7s %7s %7s %7s  %s%s\n",
           col::C, "Test name",
           "N","Mean","Std","Min","p50","p90","p99","p99.9","Max","Verdict",col::R);
    printf("%-36s %7s %7s %7s %7s %7s %7s %7s %7s %7s\n",
           "", "","(µs)","(µs)","(µs)","(µs)","(µs)","(µs)","(µs)","(µs)");
    printf("%s", line);

    bool all_pass = true;
    for (const auto& r : rows_) {
      if (r.name == "---") { printf("%s", line); continue; }
      if (r.n == 0)        { printf("  %s  (no data)\n", r.name.c_str()); continue; }
      const char* vc = VC(r.verdict);
      if (r.verdict[0] != 'P') all_pass = false;
      printf("  %-34s %7" PRIu64 " %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f  %s%-4s%s",
             r.name.c_str(), r.n,
             r.mean, r.std, r.min,
             r.p50, r.p90, r.p99, r.p999, r.max,
             vc, r.verdict, col::R);
      if (r.overflow > 0)
        printf("  %s[+%" PRIu64 " overflow >20ms]%s", col::E, r.overflow, col::R);
      printf("\n");
    }
    printf("%s", line);
    printf(all_pass ? "%s[OK] Overall: PASS%s\n\n" : "%s[!!] Overall: FAIL/WARN%s\n\n",
           all_pass ? col::G : col::E, col::R);
  }

  void WriteCsv(const char* path) const {
    FILE* f = fopen(path, "w");
    if (!f) { fprintf(stderr, "[report] cannot open %s\n", path); return; }
    fprintf(f, "test_name,n,mean_us,std_us,min_us,p50_us,p90_us,p99_us,p999_us,max_us,overflow,verdict\n");
    for (const auto& r : rows_) {
      if (r.name=="---" || r.n==0) continue;
      fprintf(f, "%s,%" PRIu64 ",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%" PRIu64 ",%s\n",
              r.name.c_str(), r.n,
              r.mean, r.std, r.min, r.p50, r.p90, r.p99, r.p999, r.max,
              r.overflow, r.verdict);
    }
    fclose(f);
    printf("[report] CSV -> %s\n", path);
  }

  /*
   * Histogram: print the distribution of a LatencySample.
   * Groups buckets into 100µs bands for readability.
   */
  static void PrintHist(const LatencySample& s, size_t max_bands = 30) {
    using H = Histogram<>;
    uint64_t tot = s.histogram.total.load();
    if (!tot) { printf("  (no data)\n"); return; }

    // Group 1µs buckets into 100µs bands
    constexpr size_t kBandWidth = 100;  // buckets per band
    constexpr size_t kNumBands  = H::kBucketCount / kBandWidth;

    uint64_t band_counts[kNumBands] = {};
    for (size_t i = 0; i < H::kBucketCount; ++i)
      band_counts[i / kBandWidth] += s.histogram.counts[i].load();

    // Find peak for scaling
    uint64_t peak = *std::max_element(band_counts, band_counts + kNumBands);
    if (!peak) return;

    printf("\n%sHistogram: %s%s  (1 bar = 100µs band)\n", col::B, s.name.c_str(), col::R);
    printf("  %6s  %-40s  %8s  %6s\n", "ms", "", "count", "%");

    size_t shown = 0;
    for (size_t b = 0; b < kNumBands && shown < max_bands; ++b) {
      if (!band_counts[b]) continue;
      ++shown;
      double pct = 100.0 * (double)band_counts[b] / (double)tot;
      int len = (int)((double)band_counts[b] / (double)peak * 40.0);
      printf("  %5.1f  %s%-*s%s  %8" PRIu64 "  %5.1f%%\n",
             (double)b * kBandWidth * 1e-3,
             col::C, std::max(len,1), std::string(std::max(len,1),'#').c_str(), col::R,
             band_counts[b], pct);
    }
    uint64_t ov = s.histogram.overflow.load();
    if (ov)
      printf("  %5s  %s%-40s%s  %8" PRIu64 "  %5.1f%%  <- >20ms\n",
             ">20", col::E, std::string(40,'!').c_str(), col::R,
             ov, 100.0*(double)ov/(double)tot);
  }

 private:
  std::string      title_;
  std::vector<Row> rows_;
};

}  // namespace bench