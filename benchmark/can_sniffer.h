/*
 * @file  can_sniffer.h
 * @brief Raw CAN socket sniffer for wire-accurate TX/RX timestamping.
 *
 * WHY THIS EXISTS
 * ───────────────
 * Polling GetPosition() measures "command written → GetPosition() returns
 * new value", which includes sleep granularity (50 µs steps) and can miss
 * frames entirely.  This class opens a SECOND raw socket on the same
 * interface and uses:
 *
 *   CAN_RAW_RECV_OWN_MSGS = 1   → also receives frames we transmit
 *   SO_TIMESTAMP              → kernel software RX timestamp (µs accuracy)
 *
 * This gives timestamps at the kernel network layer, independent of the
 * driver implementation and without modifying any production code.
 *
 * HOW IT WORKS
 * ────────────
 * Robstride protocol:
 *   TX frame (MIT control):  comm_type = bits[31:24] = 0x01
 *                             motor_id  = bits[ 7: 0]
 *   RX feedback:             comm_type = bits[31:24] = 0x02
 *                             motor_id  = bits[15: 8]  ← KEY
 *
 * The sniffer thread:
 *   1. Reads frames from the raw socket (own + peer).
 *   2. For comm_type=0x01: tx_stamp_ns[motor_id] = SO_TIMESTAMP value.
 *   3. For comm_type=0x02: rtl = SO_TIMESTAMP - tx_stamp_ns[motor_id].
 *      Adds sample to per-motor LatencySample.
 *
 * ACCURACY
 * ────────
 * SO_TIMESTAMP is set by the kernel at the moment the frame is queued
 * into the socket receive buffer.  The delta between TX and RX stamps
 * therefore equals:
 *
 *   RTL = CAN bus transmission time + motor processing time + CAN RX time
 *       ≈ (114 bits / 1 Mbps) + (motor firmware) + (114 bits / 1 Mbps)
 *       ≈ 114 µs + firmware + 114 µs
 *       ≈ 300–600 µs for Robstride 00/02
 *
 * USAGE
 * ─────
 *   CanSniffer sniffer("can0");
 *   sniffer.Start();
 *   // ... run test ...
 *   sniffer.Stop();
 *   const LatencySample& s = sniffer.RTL(motor_id);  // id 1–4
 */

#pragma once

#include <array>
#include <atomic>
#include <cstring>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "stats.h"

namespace bench {

class CanSniffer {
 public:
  static constexpr size_t kMaxIds = 8;   // supports can_id 1–8
  static constexpr uint8_t COMM_MIT      = 0x01;
  static constexpr uint8_t COMM_FEEDBACK = 0x02;

  explicit CanSniffer(const std::string& iface) : iface_(iface) {
    for (size_t i = 0; i < kMaxIds; ++i) {
      rtl_[i].name = iface + "/id" + std::to_string(i+1) + "_rtl";
      tx_ns_[i].store(0, std::memory_order_relaxed);
    }
  }

  ~CanSniffer() { Stop(); }

  bool Start() {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ < 0) {
      perror("[sniffer] socket()");
      return false;
    }

    // Receive own transmitted frames too
    int enable = 1;
    if (setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                   &enable, sizeof(enable)) < 0) {
      perror("[sniffer] CAN_RAW_RECV_OWN_MSGS");
      ::close(sockfd_); sockfd_ = -1; return false;
    }

    // Kernel software RX timestamp (µs accuracy via gettimeofday)
    if (setsockopt(sockfd_, SOL_SOCKET, SO_TIMESTAMP,
                   &enable, sizeof(enable)) < 0) {
      perror("[sniffer] SO_TIMESTAMP");
      ::close(sockfd_); sockfd_ = -1; return false;
    }

    struct ifreq ifr{};
    strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
      perror("[sniffer] SIOCGIFINDEX");
      ::close(sockfd_); sockfd_ = -1; return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      perror("[sniffer] bind()");
      ::close(sockfd_); sockfd_ = -1; return false;
    }

    running_.store(true);
    thread_ = std::thread(&CanSniffer::Loop, this);
    return true;
  }

  void Stop() {
    running_.store(false, std::memory_order_release);
    // select() in Loop() has a 100ms timeout, so thread wakes up,
    // sees running_==false and exits cleanly — no need to close sockfd_ here.
    if (thread_.joinable()) thread_.join();
    if (sockfd_ >= 0) { ::close(sockfd_); sockfd_ = -1; }
  }

  void Reset() {
    for (size_t i = 0; i < kMaxIds; ++i) {
      rtl_[i].Reset();
      tx_ns_[i].store(0, std::memory_order_relaxed);
    }
  }

  // motor_id is 1-indexed (CAN node id)
  const LatencySample& RTL(uint8_t motor_id) const {
    return rtl_[Idx(motor_id)];
  }

  const std::string& Iface() const { return iface_; }

 private:
  static size_t Idx(uint8_t id) {
    size_t i = (size_t)(id - 1);
    return (i < kMaxIds) ? i : 0;
  }

  void Loop() {
    pthread_setname_np(pthread_self(), "bench_sniff");

    struct can_frame frame{};
    struct iovec  iov{ &frame, sizeof(frame) };
    char          cmsg_buf[256];
    struct msghdr msg{};
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = cmsg_buf;
    msg.msg_controllen = sizeof(cmsg_buf);

    while (running_.load(std::memory_order_acquire)) {

      // Wait up to 100ms for a frame.
      // If no frame arrives (e.g. motor not connected), loop wakes up and
      // re-checks running_ — this is what makes Stop() non-blocking.
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(sockfd_, &fds);
      struct timeval tv_sel{ 0, 100'000 };   // 100 ms
      int rc = ::select(sockfd_ + 1, &fds, nullptr, nullptr, &tv_sel);
      if (rc < 0) break;          // socket closed or signal
      if (rc == 0) continue;      // timeout – re-check running_

      // Frame ready: read with control message for SO_TIMESTAMP
      msg.msg_controllen = sizeof(cmsg_buf);
      ssize_t n = recvmsg(sockfd_, &msg, MSG_DONTWAIT);
      if (n <= 0) continue;       // EAGAIN or transient error

      // Extract SO_TIMESTAMP (struct timeval)
      struct timeval tv{};
      for (struct cmsghdr* cm = CMSG_FIRSTHDR(&msg); cm;
           cm = CMSG_NXTHDR(&msg, cm)) {
        if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SO_TIMESTAMP) {
          memcpy(&tv, CMSG_DATA(cm), sizeof(tv));
          break;
        }
      }
      int64_t stamp_ns = (int64_t)tv.tv_sec * 1'000'000'000LL
                       + (int64_t)tv.tv_usec * 1'000LL;

      // Decode Robstride EFF frame
      if (!(frame.can_id & CAN_EFF_FLAG)) continue;
      uint32_t id_val   = frame.can_id & CAN_EFF_MASK;
      uint8_t  comm     = (uint8_t)((id_val >> 24) & 0xFF);
      uint8_t  motor_tx = (uint8_t)( id_val        & 0xFF);   // MIT TX: motor_id at [7:0]
      uint8_t  motor_rx = (uint8_t)((id_val >>  8) & 0xFF);   // Feedback: motor_id at [15:8]

      if (comm == COMM_MIT) {
        tx_ns_[Idx(motor_tx)].store(stamp_ns, std::memory_order_release);

      } else if (comm == COMM_FEEDBACK) {
        int64_t tx = tx_ns_[Idx(motor_rx)].load(std::memory_order_acquire);
        if (tx == 0) continue;
        int64_t rtl = stamp_ns - tx;
        if (rtl > 50'000LL && rtl < 20'000'000LL)
          rtl_[Idx(motor_rx)].Add(rtl);
      }
    }
  }

  std::string              iface_;
  int                      sockfd_ = -1;
  std::atomic<bool>        running_{false};
  std::thread              thread_;
  std::array<LatencySample,   kMaxIds> rtl_{};
  std::array<std::atomic<int64_t>, kMaxIds> tx_ns_{};
};

}  // namespace bench