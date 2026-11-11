#pragma once

#include "canfd_bus_base.h"
#include <string>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#ifndef MAX_MOTOR_NUM
static constexpr int kMAX_MOTOR_NUM = 7;
#else
static constexpr int kMAX_MOTOR_NUM = MAX_MOTOR_NUM;
#endif

class SocketCANBus : public CanFdBusBase {
public:
  struct FBFrame {
    uint32_t master_id{};
    uint8_t data[8]{};
    bool valid{false};
    std::chrono::steady_clock::time_point stamp{};
  };

  SocketCANBus(const std::string &interface_name, bool fd_mode = false);
  ~SocketCANBus() override;

  int Init();
  void Shutdown();
  void SetReceiverId(uint32_t *id, uint8_t len);
  void Sample();
  int Send(uint32_t id, const uint8_t *data, uint8_t len) override;
  int Receive(uint32_t id, uint8_t *data) override;

private:
  FBFrame fb_frames_[kMAX_MOTOR_NUM]{};
  std::mutex fb_frames_mutex_;
  std::jthread sample_thread_;
  std::string can_interface_;
  int socket_fd_{-1};
  bool fd_mode_;
};
