/*
 * @file  bench_main.cpp
 * @brief Robstride SocketCAN driver – accurate latency benchmark.
 *
 * MEASUREMENT METHOD
 * ──────────────────
 * All RTL measurements use CanSniffer (can_sniffer.h):
 *   • Opens a second raw socket on the same interface.
 *   • CAN_RAW_RECV_OWN_MSGS=1  → sees both outgoing (TX) and incoming (RX) frames.
 *   • SO_TIMESTAMP              → kernel software timestamp at socket receive.
 *   • RTL = timestamp(feedback frame) − timestamp(MIT control frame)
 *           for the SAME motor_id.
 *
 * This removes all polling artefacts from the previous version.
 *
 * TOPOLOGY
 * ────────
 *   Bus 0 (can0): "a" id=1 Robstride_00 | "b" id=2 Robstride_02
 *   Bus 1 (can1): "c" id=1 Robstride_00 | "d" id=2 Robstride_02
 *   Bus 2 (can2): "e" id=3 Robstride_02 | "f" id=4 Robstride_00
 *   Bus 3 (can3): "g" id=3 Robstride_02 | "h" id=4 Robstride_00
 *
 * TESTS
 * ─────
 *   T1  RTL on can0 (bus 0, motors a/b)           – requires hardware
 *   T2  Control-loop jitter SCHED_OTHER vs FIFO   – software only
 *   T3  Full-pipeline cmd latency (SetMitCmd→RX)  – requires hardware
 *   T4  All 4 buses simultaneous RTL              – requires hardware + --t4
 *   T5  send_mtx_ contention model (8-motor load) – software only
 *
 * BUILD
 * ─────
 *   cmake -B build -S . && cmake --build build
 *
 * RUN
 * ───
 *   sudo ./build/benchmark/bench --duration 30
 *   sudo ./build/benchmark/bench --duration 30 --t4 --histogram
 *
 * FLAGS
 *   --duration <s>    Per-test duration in seconds (default 10)
 *   --cycle-ns <ns>   Control-loop period (default 1000000)
 *   --rt-prio  <n>    SCHED_FIFO priority (default 80)
 *   --csv <path>      CSV output (default bench_results.csv)
 *   --t4              Enable 4-bus stress test
 *   --histogram       Print per-test ASCII histogram
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <inttypes.h>
#include <pthread.h>
#include <sched.h>

#include "xyber_controller.h"
#include "stats.h"
#include "latency_probe.h"
#include "can_sniffer.h"
#include "report.h"

using namespace xyber;
using namespace bench;

// ════════════════════════════════════════════════════════════════════════════
//  Motor topology (mirrors main.cpp exactly)
// ════════════════════════════════════════════════════════════════════════════

struct MotorDef {
  uint8_t bus; ActuatorType type; const char* name; uint8_t can_id;
};

static const MotorDef kMotors[] = {
  { 0, ActuatorType::Robstride_00, "a", 1 },
  { 0, ActuatorType::Robstride_02, "b", 2 },
  { 1, ActuatorType::Robstride_00, "c", 1 },
  { 1, ActuatorType::Robstride_02, "d", 2 },
  { 2, ActuatorType::Robstride_02, "e", 3 },
  { 2, ActuatorType::Robstride_00, "f", 4 },
  { 3, ActuatorType::Robstride_02, "g", 3 },
  { 3, ActuatorType::Robstride_00, "h", 4 },
};
static constexpr int kN = (int)(sizeof(kMotors)/sizeof(kMotors[0]));

// ════════════════════════════════════════════════════════════════════════════
//  Config
// ════════════════════════════════════════════════════════════════════════════

struct Config {
  int      duration_s = 10;
  uint64_t cycle_ns   = 1'000'000;
  int      rt_prio    = 80;
  const char* csv     = "bench_results.csv";
  bool     run_t4     = false;
  bool     show_hist  = false;
};
static Config G;

// ════════════════════════════════════════════════════════════════════════════
//  Helpers
// ════════════════════════════════════════════════════════════════════════════

static void SetRT(pthread_t tid, int prio) {
  struct sched_param sp{}; sp.sched_priority = prio;
  pthread_setschedparam(tid, SCHED_FIFO, &sp);
}

// Attach only motors on a given bus to the device
static void AttachBus(XyberController* ctrl, const char* dev, uint8_t bus) {
  for (const auto& m : kMotors)
    if (m.bus == bus)
      ctrl->AttachActuator(dev, bus, m.type, m.name, m.can_id);
}

// Attach all motors
static void AttachAll(XyberController* ctrl, const char* dev) {
  for (const auto& m : kMotors)
    ctrl->AttachActuator(dev, m.bus, m.type, m.name, m.can_id);
}

// Enable and wait for firmware RUNNING state
static bool EnableAndWait(XyberController* ctrl, const char* name,
                          int timeout_ms = 300) {
  ctrl->EnableActuator(name);
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (ctrl->GetPowerState(name) == STATE_ENABLE) return true;
  }
  return false;  // no hardware or not responding
}

// Print separator header
static void Header(const char* title) {
  printf("\n=== %s ===\n", title);
}

// ════════════════════════════════════════════════════════════════════════════
//  T1 – Wire-accurate Round-Trip Latency via CanSniffer
// ════════════════════════════════════════════════════════════════════════════
/*
 * CanSniffer opens a raw socket alongside the driver.
 * CAN_RAW_RECV_OWN_MSGS captures our own MIT frames (comm_type=0x01) → TX stamp.
 * Robstride feedback (comm_type=0x02) → RX stamp.
 * RTL = RX stamp − TX stamp, computed inside CanSniffer::Loop().
 *
 * The driver continues to run normally; sniffer is read-only and transparent.
 *
 * Motor "a" (id=1) and "b" (id=2) on can0.
 */
static void RunT1(Report& rpt) {
  Header("T1: Wire RTL via CanSniffer (can0, motors a/b)");

  CanSniffer sniffer("can0");
  if (!sniffer.Start()) {
    printf("  [T1] Cannot open sniffer socket – skipping.\n");
    return;
  }

  auto* ctrl = XyberController::GetInstance();
  ctrl->CreateDevice("t1", {"can0","","",""});
  ctrl->AttachActuator("t1", 0, ActuatorType::Robstride_00, "t1_a", 1);
  ctrl->AttachActuator("t1", 0, ActuatorType::Robstride_02, "t1_b", 2);
  ctrl->Start(G.cycle_ns);

  // 100ms: firmware boot delay (must not send commands before RUNNING state)
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool hw_a = EnableAndWait(ctrl, "t1_a");
  bool hw_b = EnableAndWait(ctrl, "t1_b");
  if (!hw_a && !hw_b) {
    printf("  [T1] No motors responded – skipping (no hardware).\n");
    sniffer.Stop(); ctrl->Stop(); return;
  }
  // Extra 50ms to stabilise after enable
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  sniffer.Reset();  // clear any warm-up frames

  printf("  Running for %d s...\n", G.duration_s);
  const int64_t end = NowNs() + (int64_t)G.duration_s * 1'000'000'000LL;
  float pos = 0.0f;
  while (NowNs() < end) {
    pos = (pos > 0.3f) ? -0.3f : 0.3f;
    if (hw_a) ctrl->SetMitCmd("t1_a", pos, 0.0f, 0.0f, 10.0f, 0.5f);
    if (hw_b) ctrl->SetMitCmd("t1_b", pos, 0.0f, 0.0f, 10.0f, 0.5f);
    // Run at control-loop rate – sniffer timestamps asynchronously
    SleepUntil(NowNs() + (int64_t)G.cycle_ns);
  }

  ctrl->DisableAllActuator();
  ctrl->Stop();
  sniffer.Stop();

  if (hw_a) {
    rpt.Add(sniffer.RTL(1), kRTLThresh);
    if (G.show_hist) Report::PrintHist(sniffer.RTL(1));
  }
  if (hw_b) {
    rpt.Add(sniffer.RTL(2), kRTLThresh);
    if (G.show_hist) Report::PrintHist(sniffer.RTL(2));
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  T2 – Control-Loop Jitter
// ════════════════════════════════════════════════════════════════════════════
/*
 * No hardware required.
 *
 * Measures |actual_wakeup_period − target_cycle_ns|.
 * Uses kPeriodThresh for period rows (not kJitterThresh).
 */
static void RunT2(Report& rpt) {
  Header("T2: Control-Loop Jitter");
  printf("  cycle=%" PRIu64 " ns  (%.0f Hz)\n",
         G.cycle_ns, 1e9/(double)G.cycle_ns);

  auto run_one = [&](const char* label, bool use_rt) {
    ControlLoopProbe probe(label, G.cycle_ns);
    std::atomic_bool go{true};

    std::thread t([&](){
      if (use_rt) SetRT(pthread_self(), G.rt_prio);
      int64_t next = NowNs();
      // 200-cycle warm-up
      for (int w=0; w<200; ++w) { next += (int64_t)G.cycle_ns; SleepUntil(next); }
      next = NowNs();
      while (go.load(std::memory_order_relaxed)) {
        probe.Tick();
        // Simulate ~3µs TransmitAll (2 motors × CanFrame write)
        volatile int d=0; for(int i=0;i<300;++i)d+=i; (void)d;
        next += (int64_t)G.cycle_ns;
        SleepUntil(next);
      }
    });

    std::this_thread::sleep_for(std::chrono::seconds(G.duration_s));
    go.store(false);
    t.join();

    // Jitter = deviation from target → use kJitterThresh
    rpt.Add(probe.JitterSample(), kJitterThresh);
    // Period = absolute period value → use kPeriodThresh (deviation from 1000µs)
    rpt.Add(probe.PeriodSample(), kPeriodThresh);
    if (G.show_hist) {
      Report::PrintHist(probe.JitterSample());
      Report::PrintHist(probe.PeriodSample());
    }
  };

  run_one("t2a_SCHED_OTHER", false);
  rpt.Sep();
  run_one("t2b_SCHED_FIFO",  true);
}

// ════════════════════════════════════════════════════════════════════════════
//  T3 – Full-Pipeline Command Latency
// ════════════════════════════════════════════════════════════════════════════
/*
 * Measures from the moment SetMitCmd() is called in user code to the moment
 * the sniffer sees the corresponding feedback frame on the wire.
 *
 * Timeline:
 *   t0 = NowNs()
 *   SetMitCmd("t3_a", ...)        ← writes to send_buf_ slot
 *   [control loop fires at next cycle_ns tick → TransmitAll() → socket TX]
 *   [motor processes frame → sends feedback]
 *   [sniffer sees feedback → stamps RX]
 *   t1 = sniffer RX stamp for motor id=1
 *
 * pipeline_latency = t1 - t0
 *   = (wait for next TX cycle) + CAN TX time + motor processing + CAN RX time
 *   Expected: ≤ cycle_ns + RTL ≈ 1ms + 300-600µs ≈ 1.3-1.6ms
 */
static void RunT3(Report& rpt) {
  Header("T3: Full-Pipeline Command Latency (can0, motor 'a' id=1)");
  printf("  Expected: <= cycle_ns + RTL ~= %.0f + RTL µs\n",
         (double)G.cycle_ns * 1e-3);

  CanSniffer sniffer("can0");
  if (!sniffer.Start()) {
    printf("  [T3] Cannot open sniffer socket – skipping.\n");
    return;
  }

  auto* ctrl = XyberController::GetInstance();
  ctrl->CreateDevice("t3", {"can0","","",""});
  ctrl->AttachActuator("t3", 0, ActuatorType::Robstride_00, "t3_a", 1);
  ctrl->Start(G.cycle_ns);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (!EnableAndWait(ctrl, "t3_a")) {
    printf("  [T3] Motor not responding – skipping (no hardware).\n");
    sniffer.Stop(); ctrl->Stop(); return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  LatencySample pipeline;
  pipeline.name = "t3_pipeline_latency_a";
  uint64_t timeouts = 0;

  const int64_t end = NowNs() + (int64_t)G.duration_s * 1'000'000'000LL;
  float pos = 0.0f;
  while (NowNs() < end) {
    pos = (pos > 0.3f) ? -0.3f : 0.3f;

    // Record time immediately before command
    const int64_t t0 = NowNs();
    // Remember last RX stamp seen by sniffer for motor 1
    const uint64_t n_before = sniffer.RTL(1).stats.n;

    ctrl->SetMitCmd("t3_a", pos, 0.0f, 0.0f, 10.0f, 0.5f);

    // Wait until sniffer receives at least one NEW feedback frame
    // Timeout: 2 × cycle_ns + 5ms margin
    const int64_t deadline = t0 + (int64_t)G.cycle_ns * 2 + 5'000'000LL;
    bool got_fb = false;
    while (NowNs() < deadline) {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      if (sniffer.RTL(1).stats.n > n_before) { got_fb = true; break; }
    }

    if (!got_fb) { ++timeouts; }
    else         { pipeline.Add(NowNs() - t0); }

    SleepUntil(NowNs() + (int64_t)G.cycle_ns);
  }

  ctrl->DisableActuator("t3_a");
  ctrl->Stop();
  sniffer.Stop();

  rpt.Add(pipeline, kRTLThresh);
  const uint64_t total = pipeline.stats.n + timeouts;
  if (timeouts > 0)
    printf("  [T3] Timeouts: %" PRIu64 "/%" PRIu64 " (%.1f%%)\n",
           timeouts, total, 100.0*(double)timeouts/(double)std::max((uint64_t)1, total));
  if (G.show_hist) Report::PrintHist(pipeline);
}

// ════════════════════════════════════════════════════════════════════════════
//  T4 – Multi-Bus Stress (all 4 buses + all 8 motors)
// ════════════════════════════════════════════════════════════════════════════
/*
 * Requires 4 physical CAN buses.  Enable with --t4.
 * One CanSniffer per bus.  Reports RTL for every motor.
 */
static void RunT4(Report& rpt) {
  if (!G.run_t4) {
    Header("T4: Multi-Bus Stress (SKIPPED – use --t4)");
    return;
  }
  Header("T4: Multi-Bus Stress (4 buses, 8 motors)");

  // One sniffer per bus
  CanSniffer s0("can0"), s1("can1"), s2("can2"), s3("can3");
  CanSniffer* sniffers[4] = {&s0, &s1, &s2, &s3};
  for (auto* s : sniffers) if (!s->Start()) {
    printf("  [T4] Sniffer start failed – skipping.\n"); return;
  }

  auto* ctrl = XyberController::GetInstance();
  ctrl->CreateDevice("t4", {"can0","can1","can2","can3"});
  AttachAll(ctrl, "t4");
  ctrl->Start(G.cycle_ns);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ctrl->EnableAllActuator();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (auto* s : sniffers) s->Reset();

  // User thread: hammer all motors at 2× loop rate
  std::atomic_bool stress{true};
  std::thread st([&](){
    float pos = 0.0f;
    while (stress.load(std::memory_order_relaxed)) {
      pos = (pos > 0.3f) ? -0.3f : 0.3f;
      for (const auto& m : kMotors)
        ctrl->SetMitCmd(m.name, pos, 0.0f, 0.0f, 5.0f, 0.3f);
      SleepUntil(NowNs() + (int64_t)G.cycle_ns/2);
    }
  });

  std::this_thread::sleep_for(std::chrono::seconds(G.duration_s));
  stress.store(false);
  st.join();
  ctrl->DisableAllActuator();
  ctrl->Stop();
  for (auto* s : sniffers) s->Stop();

  for (int b = 0; b < 4; ++b) {
    for (const auto& m : kMotors) {
      if (m.bus != (uint8_t)b) continue;
      rpt.Add(sniffers[b]->RTL(m.can_id), kRTLThresh);
      if (G.show_hist) Report::PrintHist(sniffers[b]->RTL(m.can_id));
    }
    rpt.Sep();
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  T5 – Mutex Contention (8-motor workload model)
// ════════════════════════════════════════════════════════════════════════════
/*
 * No hardware required.
 * Models the actual lock pattern with 8 motors:
 *   ctrl loop  : holds send_mtx_ for ~2µs (8 CanFrame copies) every cycle_ns
 *   user thread: tries to acquire send_mtx_ for 8 × SetMitCmd at 2× rate
 */
static void RunT5(Report& rpt) {
  Header("T5: send_mtx_ Contention (8-motor model)");

  std::mutex mtx;
  MutexProbe probe("t5_send_mtx");
  std::atomic_bool go{true};

  // Ctrl loop: holds for ~2µs
  std::thread ctrl_t([&](){
    SetRT(pthread_self(), G.rt_prio);
    int64_t next = NowNs();
    while (go.load(std::memory_order_relaxed)) {
      { std::lock_guard<std::mutex> lk(mtx);
        volatile int d=0; for(int i=0;i<200;++i)d+=i; (void)d; }  // ~2µs
      next += (int64_t)G.cycle_ns;
      SleepUntil(next);
    }
  });

  // User: 8 motors × SetMitCmd at 2× rate
  std::thread user_t([&](){
    int64_t next = NowNs();
    while (go.load(std::memory_order_relaxed)) {
      { int64_t ws = probe.BeginWait();
        std::lock_guard<std::mutex> lk(mtx);
        probe.Acquired(ws);
        // 8 motors × FloatToUint16 × 5 params ≈ 8 × 200ns = ~1.6µs
        volatile float f=1.0f;
        for(int i=0;i<160;++i) f=f*1.0001f+0.0001f; (void)f;
      }
      probe.Released();
      next += (int64_t)G.cycle_ns/2;
      SleepUntil(next);
    }
  });

  std::this_thread::sleep_for(std::chrono::seconds(G.duration_s));
  go.store(false);
  ctrl_t.join();
  user_t.join();

  rpt.Add(probe.WaitSample(), kMutexThresh);
  rpt.Add(probe.HeldSample(), kMutexThresh);
  if (G.show_hist) {
    Report::PrintHist(probe.WaitSample());
    Report::PrintHist(probe.HeldSample());
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Args + main
// ════════════════════════════════════════════════════════════════════════════

static void ParseArgs(int argc, char** argv) {
  for (int i=1; i<argc; ++i) {
    if (!strcmp(argv[i],"--duration")  && i+1<argc) G.duration_s = atoi(argv[++i]);
    if (!strcmp(argv[i],"--cycle-ns")  && i+1<argc) G.cycle_ns   = (uint64_t)atoll(argv[++i]);
    if (!strcmp(argv[i],"--rt-prio")   && i+1<argc) G.rt_prio    = atoi(argv[++i]);
    if (!strcmp(argv[i],"--csv")       && i+1<argc) G.csv        = argv[++i];
    if (!strcmp(argv[i],"--t4"))        G.run_t4    = true;
    if (!strcmp(argv[i],"--histogram")) G.show_hist = true;
  }
  G.duration_s = std::max(1, G.duration_s);
}

int main(int argc, char** argv) {
  ParseArgs(argc, argv);

  printf("\n=== Robstride SocketCAN – Latency Benchmark ===\n\n");
  printf("  Motor topology:\n");
  for (const auto& m : kMotors)
    printf("    can%u  id=%u  %-12s  %s\n", m.bus, m.can_id,
           m.type==ActuatorType::Robstride_00 ? "Robstride_00" : "Robstride_02",
           m.name);
  printf("\n  Duration  : %d s/test\n", G.duration_s);
  printf("  Cycle     : %" PRIu64 " ns  (%.0f Hz)\n",
         G.cycle_ns, 1e9/(double)G.cycle_ns);
  printf("  RTL thres : PASS<%.0fµs  WARN<%.0fµs\n",
         kRTLThresh.pass_p99_us, kRTLThresh.warn_p99_us);
  printf("  T4 stress : %s\n\n", G.run_t4 ? "enabled" : "disabled (--t4)");

  Report rpt("Robstride SocketCAN – Benchmark Results");

  RunT1(rpt);  rpt.Sep();
  RunT2(rpt);  rpt.Sep();
  RunT3(rpt);  rpt.Sep();
  RunT4(rpt);  rpt.Sep();
  RunT5(rpt);

  rpt.Print();
  rpt.WriteCsv(G.csv);
  return 0;
}