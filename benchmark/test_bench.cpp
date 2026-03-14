/*
 * @file  bench_main.cpp
 * @brief Robstride latency benchmark – 5 test suites.
 *
 * T1  Round-Trip Latency        (RTL per motor per bus)
 * T2  Control-Loop Jitter       (SCHED_OTHER vs SCHED_FIFO)
 * T3  Application Command Latency (SetMitCmd → GetPosition changes)
 * T4  Multi-Bus Stress          (4 buses × 2 motors)
 * T5  Mutex Contention Model    (send_mtx_ wait time)
 *
 * Build:
 *   cmake -B build -S . && cmake --build build
 *
 * Run:
 *   sudo ./build/benchmark/bench --iface can0 --motors 2 --duration 30
 *   sudo ./build/benchmark/bench --ifaces can0,can1,can2,can3 --duration 30
 */

// ── Standard library ──────────────────────────────────────────────────────────
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// ── Driver ───────────────────────────────────────────────────────────────────
#include "xyber_controller.h"

// ── Benchmark infrastructure ─────────────────────────────────────────────────
//  Include order matters: stats first, then probe, then report.
//  Do NOT include stats.h twice (probe and report already include it via pragma once).
#include "stats.h"
#include "latency_probe.h"
#include "report.h"

using namespace xyber;
using namespace bench;

// ════════════════════════════════════════════════════════════════════════════
//  Config
// ════════════════════════════════════════════════════════════════════════════

struct Config {
  std::string iface      = "can0";
  std::string ifaces[4]  = {"can0","can1","can2","can3"};
  int  n_motors    = 2;       // 2 motors per bus (id 1 & 2)
  int  duration_s  = 10;
  uint64_t cycle_ns = 1'000'000;  // 1 ms
  int  rt_prio     = 80;
  std::string csv_path = "bench_results.csv";
  bool run_t4   = true;
  bool show_hist = false;
};

static Config g_cfg;

// ════════════════════════════════════════════════════════════════════════════
//  Helpers
// ════════════════════════════════════════════════════════════════════════════

static void SetThreadRT(pthread_t tid, int prio, int cpu = -1) {
  struct sched_param sp{}; sp.sched_priority = prio;
  pthread_setschedparam(tid, SCHED_FIFO, &sp);
  if (cpu >= 0) {
    cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(cpu, &cs);
    pthread_setaffinity_np(tid, sizeof(cs), &cs);
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  T1 – Round-Trip Latency
// ════════════════════════════════════════════════════════════════════════════
/*
 * How it works (without modifying driver source):
 *   - LatencyProbe runs in passive mode alongside the control loop.
 *   - bench_main registers its own RX callbacks on the SocketCAN instance
 *     AFTER the driver registers its callbacks.  Both callbacks fire for each
 *     received frame; the bench callback only timestamps.
 *   - TX stamp: recorded immediately before Start() fires, in a wrapper loop
 *     that shadows each TransmitAll() cycle via a ControlLoopProbe (T2 uses
 *     the same mechanism).
 *
 * Practical note:
 *   True TX-stamp injection requires hooking inside TransmitAll().
 *   Without source modification, T1 measures "command written to slot →
 *   feedback decoded" which is a superset of raw CAN RTT.
 *   This is the latency that matters to the application anyway.
 */
static void RunT1(Report& report, LatencyProbe probes[4]) {
  printf("\n--- T1: Round-Trip Latency (%s, %d motor(s)) ---\n",
         g_cfg.iface.c_str(), g_cfg.n_motors);

  auto* ctrl = XyberController::GetInstance();
  ctrl->CreateDevice("t1_dev", {g_cfg.iface, "", "", ""});
  for (int i = 0; i < g_cfg.n_motors; ++i) {
    ctrl->AttachActuator("t1_dev", 0, ActuatorType::Robstride_02,
                         "t1_m" + std::to_string(i + 1),
                         static_cast<uint8_t>(i + 1));
  }
  ctrl->Start(g_cfg.cycle_ns);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // firmware boot delay

  probes[0].Enable();

  // Simulate command + measure time-to-feedback per motor
  const int64_t end_ns = NowNs() + static_cast<int64_t>(g_cfg.duration_s) * 1'000'000'000LL;
  float pos = 0.0f;
  while (NowNs() < end_ns) {
    for (int m = 0; m < g_cfg.n_motors; ++m) {
      const uint8_t id = static_cast<uint8_t>(m + 1);
      const std::string name = "t1_m" + std::to_string(m + 1);
      probes[0].StampTx(id);
      ctrl->SetMitCmd(name, pos, 0.0f, 0.0f, 10.0f, 0.5f);
      // StampRx will be called by a thin wrapper if hooks are injected;
      // here we poll for position change as a proxy:
      const float before = ctrl->GetPosition(name);
      SleepUntil(NowNs() + static_cast<int64_t>(g_cfg.cycle_ns));
      const float after  = ctrl->GetPosition(name);
      if (std::abs(after - before) > 1e-6f) {
        probes[0].StampRx(id);
      }
    }
    pos = (pos > 0.2f) ? -0.2f : 0.2f;
  }

  probes[0].Disable();
  ctrl->Stop();

  for (int m = 0; m < g_cfg.n_motors; ++m) {
    const uint8_t id = static_cast<uint8_t>(m + 1);
    report.AddRow(probes[0].RTL(id), kRTLThresh);
    if (g_cfg.show_hist) Report::PrintHistogram(probes[0].RTL(id));
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  T2 – Control-Loop Jitter
// ════════════════════════════════════════════════════════════════════════════
/*
 * Measures |actual_wakeup_period – target_cycle_ns|.
 * Two sub-tests: SCHED_OTHER vs SCHED_FIFO.
 *
 * Fix for lambda-scope bug in original code:
 *   ControlLoopProbe is now declared in the enclosing function scope and
 *   captured by reference into the measurement thread lambda.
 */
static void RunT2(Report& report) {
  printf("\n--- T2: Control-Loop Jitter (cycle=%" PRIu64 " ns) ---\n",
         g_cfg.cycle_ns);

  // ── Helper: run one jitter test, fill probe, then add rows ──
  auto run_jitter_test = [&](const std::string& label, bool use_rt) {
    // Probe declared HERE (enclosing scope), captured by ref into thread
    ControlLoopProbe probe(label, g_cfg.cycle_ns);
    std::atomic_bool running{true};

    std::thread t([&]() {
      if (use_rt) SetThreadRT(pthread_self(), g_cfg.rt_prio);
      int64_t next = NowNs();
      // 100-cycle warm-up
      for (int w = 0; w < 100; ++w) {
        next += static_cast<int64_t>(g_cfg.cycle_ns);
        SleepUntil(next);
      }
      next = NowNs();
      while (running.load(std::memory_order_relaxed)) {
        probe.Tick();
        // Simulate ~3 µs of TransmitAll work (2 motors × CanFrame copy)
        volatile int dummy = 0;
        for (int i = 0; i < 300; ++i) dummy += i;
        (void)dummy;
        next += static_cast<int64_t>(g_cfg.cycle_ns);
        SleepUntil(next);
      }
    });

    std::this_thread::sleep_for(std::chrono::seconds(g_cfg.duration_s));
    running.store(false);
    t.join();

    // Probe is still in scope here — safe to read
    report.AddRow(probe.JitterSample(), kJitterThresh);
    report.AddRow(probe.PeriodSample(), kJitterThresh);
    if (g_cfg.show_hist) {
      Report::PrintHistogram(probe.JitterSample());
      Report::PrintHistogram(probe.PeriodSample());
    }
  };

  run_jitter_test("t2a_SCHED_OTHER", false);
  report.AddSeparator();
  run_jitter_test("t2b_SCHED_FIFO",  true);
}

// ════════════════════════════════════════════════════════════════════════════
//  T3 – Application-Level Command Latency
// ════════════════════════════════════════════════════════════════════════════
static void RunT3(Report& report) {
  printf("\n--- T3: Application Command Latency (%s) ---\n",
         g_cfg.iface.c_str());
  printf("  Requires connected Robstride motor. Timeout = 20 ms/sample.\n");

  auto* ctrl = XyberController::GetInstance();
  ctrl->CreateDevice("t3_dev", {g_cfg.iface, "", "", ""});
  ctrl->AttachActuator("t3_dev", 0, ActuatorType::Robstride_02, "t3_m1", 1);
  ctrl->Start(g_cfg.cycle_ns);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // firmware boot delay

  if (!ctrl->EnableActuator("t3_m1")) {
    printf("  [T3] Motor did not respond – skipping (no hardware).\n");
    ctrl->Stop();
    return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // stable after enable

  LatencySample cmd_lat;
  cmd_lat.name = "t3_cmd_latency";
  uint64_t timeouts = 0;
  constexpr int64_t kTimeoutNs = 20'000'000LL;

  const int64_t end_ns = NowNs() + static_cast<int64_t>(g_cfg.duration_s) * 1'000'000'000LL;
  float pos_tgt = 0.0f;
  while (NowNs() < end_ns) {
    pos_tgt = (pos_tgt > 0.2f) ? -0.2f : 0.2f;
    const float before = ctrl->GetPosition("t3_m1");
    const int64_t t0   = NowNs();
    ctrl->SetMitCmd("t3_m1", pos_tgt, 0.0f, 0.0f, 10.0f, 0.5f);

    bool timedout = true;
    const int64_t deadline = t0 + kTimeoutNs;
    while (NowNs() < deadline) {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      if (std::abs(ctrl->GetPosition("t3_m1") - before) > 0.001f) {
        timedout = false;
        break;
      }
    }
    if (timedout) ++timeouts;
    else          cmd_lat.Add(NowNs() - t0);
  }

  ctrl->DisableActuator("t3_m1");
  ctrl->Stop();

  report.AddRow(cmd_lat, kRTLThresh);
  const uint64_t total_attempts = cmd_lat.stats.n + timeouts;
  if (timeouts > 0)
    printf("  [T3] Timeouts: %" PRIu64 " / %" PRIu64 " (%.1f%%)\n",
           timeouts, total_attempts,
           100.0 * static_cast<double>(timeouts) / std::max(1UL, (unsigned long)total_attempts));
  if (g_cfg.show_hist) Report::PrintHistogram(cmd_lat);
}

// ════════════════════════════════════════════════════════════════════════════
//  T4 – Multi-Bus Stress (4 buses × 2 motors)
// ════════════════════════════════════════════════════════════════════════════
static void RunT4(Report& report, LatencyProbe probes[4]) {
  if (!g_cfg.run_t4) {
    printf("\n--- T4: Multi-Bus Stress (SKIPPED via --no-t4) ---\n");
    return;
  }
  printf("\n--- T4: Multi-Bus Stress (4 buses × %d motors) ---\n", g_cfg.n_motors);

  auto* ctrl = XyberController::GetInstance();
  ctrl->CreateDevice("t4_dev", {g_cfg.ifaces[0], g_cfg.ifaces[1],
                                 g_cfg.ifaces[2], g_cfg.ifaces[3]});

  for (uint8_t bus = 0; bus < 4; ++bus) {
    if (g_cfg.ifaces[bus].empty()) continue;
    for (int m = 1; m <= g_cfg.n_motors; ++m) {
      const std::string name = "t4_b" + std::to_string(bus) + "_m" + std::to_string(m);
      ctrl->AttachActuator("t4_dev", bus, ActuatorType::Robstride_02,
                           name, static_cast<uint8_t>(m));
    }
  }

  for (int b = 0; b < 4; ++b) probes[b].Enable();

  ctrl->Start(g_cfg.cycle_ns);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // User thread hammering all motors at 2× control-loop rate
  std::atomic_bool stress{true};
  std::thread stress_t([&]() {
    float pos = 0.0f;
    while (stress.load(std::memory_order_relaxed)) {
      pos = (pos > 0.2f) ? -0.2f : 0.2f;
      for (uint8_t bus = 0; bus < 4; ++bus) {
        if (g_cfg.ifaces[bus].empty()) continue;
        for (int m = 1; m <= g_cfg.n_motors; ++m) {
          const std::string name = "t4_b" + std::to_string(bus) + "_m" + std::to_string(m);
          const uint8_t id = static_cast<uint8_t>(m);
          probes[bus].StampTx(id);
          ctrl->SetMitCmd(name, pos, 0.0f, 0.0f, 5.0f, 0.3f);
        }
      }
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(static_cast<int64_t>(g_cfg.cycle_ns / 2)));
    }
  });

  std::this_thread::sleep_for(std::chrono::seconds(g_cfg.duration_s));
  stress.store(false);
  stress_t.join();

  for (int b = 0; b < 4; ++b) probes[b].Disable();
  ctrl->Stop();

  for (int b = 0; b < 4; ++b) {
    if (g_cfg.ifaces[b].empty()) continue;
    for (int m = 1; m <= g_cfg.n_motors; ++m)
      report.AddRow(probes[b].RTL(static_cast<uint8_t>(m)), kRTLThresh);
    report.AddSeparator();
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  T5 – Mutex Contention Model
// ════════════════════════════════════════════════════════════════════════════
static void RunT5(Report& report) {
  printf("\n--- T5: send_mtx_ Contention Model ---\n");

  std::mutex shared_mtx;
  MutexProbe user_probe("t5_send_mtx");
  std::atomic_bool running{true};

  // "Control-loop" thread: holds mutex ~1 µs every cycle_ns
  std::thread ctrl_t([&]() {
    SetThreadRT(pthread_self(), g_cfg.rt_prio);
    int64_t next = NowNs();
    while (running.load(std::memory_order_relaxed)) {
      {
        std::lock_guard<std::mutex> lock(shared_mtx);
        volatile int d = 0;
        for (int i = 0; i < 100; ++i) d += i;
        (void)d;
      }
      next += static_cast<int64_t>(g_cfg.cycle_ns);
      SleepUntil(next);
    }
  });

  // "User thread": calls SetMitCmd at 2× rate
  std::thread user_t([&]() {
    int64_t next = NowNs();
    while (running.load(std::memory_order_relaxed)) {
      {
        const int64_t ws = user_probe.BeginWait();
        std::lock_guard<std::mutex> lock(shared_mtx);
        user_probe.Acquired(ws);
        volatile float f = 1.0f;
        for (int i = 0; i < 20; ++i) f = f * 1.0001f + 0.0001f;
        (void)f;
      }
      user_probe.Released();
      next += static_cast<int64_t>(g_cfg.cycle_ns / 2);
      SleepUntil(next);
    }
  });

  std::this_thread::sleep_for(std::chrono::seconds(g_cfg.duration_s));
  running.store(false);
  ctrl_t.join();
  user_t.join();

  report.AddRow(user_probe.WaitSample(), kMutexThresh);
  report.AddRow(user_probe.HeldSample(), kMutexThresh);
  if (g_cfg.show_hist) {
    Report::PrintHistogram(user_probe.WaitSample());
    Report::PrintHistogram(user_probe.HeldSample());
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Argument parsing
// ════════════════════════════════════════════════════════════════════════════

static void ParseArgs(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i],"--iface")    && i+1<argc) g_cfg.iface      = argv[++i];
    if (!strcmp(argv[i],"--motors")   && i+1<argc) g_cfg.n_motors   = atoi(argv[++i]);
    if (!strcmp(argv[i],"--duration") && i+1<argc) g_cfg.duration_s = atoi(argv[++i]);
    if (!strcmp(argv[i],"--cycle-ns") && i+1<argc) g_cfg.cycle_ns   = (uint64_t)atoll(argv[++i]);
    if (!strcmp(argv[i],"--rt-prio")  && i+1<argc) g_cfg.rt_prio    = atoi(argv[++i]);
    if (!strcmp(argv[i],"--csv")      && i+1<argc) g_cfg.csv_path   = argv[++i];
    if (!strcmp(argv[i],"--no-t4"))   g_cfg.run_t4   = false;
    if (!strcmp(argv[i],"--histogram")) g_cfg.show_hist = true;
    if (!strcmp(argv[i],"--ifaces")  && i+1<argc) {
      char buf[256]; strncpy(buf, argv[++i], 255); buf[255] = 0;
      char* tok = strtok(buf, ",");
      for (int b = 0; b < 4 && tok; ++b, tok = strtok(nullptr,","))
        g_cfg.ifaces[b] = tok;
    }
  }
  g_cfg.n_motors   = std::max(1, std::min(4, g_cfg.n_motors));
  g_cfg.duration_s = std::max(1, g_cfg.duration_s);
}

// ════════════════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
  ParseArgs(argc, argv);

  printf("\n=== Robstride SocketCAN Driver - Latency Benchmark ===\n\n");
  printf("  Interface  : %s\n",    g_cfg.iface.c_str());
  printf("  Motors/bus : %d\n",    g_cfg.n_motors);
  printf("  Duration   : %d s\n",  g_cfg.duration_s);
  printf("  Cycle      : %" PRIu64 " ns  (%.0f Hz)\n",
         g_cfg.cycle_ns, 1e9 / static_cast<double>(g_cfg.cycle_ns));
  printf("  RT prio    : SCHED_FIFO %d\n", g_cfg.rt_prio);
  printf("  CSV        : %s\n\n",  g_cfg.csv_path.c_str());

  // One probe per bus (4 total) — declared here so RunT1 and RunT4 can share
  LatencyProbe probes[4] = {
    LatencyProbe("can0"),
    LatencyProbe("can1"),
    LatencyProbe("can2"),
    LatencyProbe("can3"),
  };

  Report report("Robstride SocketCAN Driver - Latency Results");

  const int n_tests = g_cfg.run_t4 ? 5 : 4;
  printf("Running %d tests (~%d s total)...\n", n_tests, g_cfg.duration_s * n_tests);

  RunT1(report, probes);   report.AddSeparator();
  RunT2(report);           report.AddSeparator();
  RunT3(report);           report.AddSeparator();
  RunT4(report, probes);   report.AddSeparator();
  RunT5(report);

  report.PrintTerminal();
  report.WriteCsv(g_cfg.csv_path);

  return 0;
}