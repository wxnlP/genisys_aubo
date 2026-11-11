#include "canfd_bus.h"
#include "socket_can.h"
#include <cstring>

namespace {
// 反馈帧有效期：500Hz 下约 25 帧，超过则视为该关节已失联
constexpr int kFrameTimeoutMs = 50;
}  // namespace


SocketCANBus::SocketCANBus(const std::string &interface_name, bool fd_mode)
    : can_interface_(interface_name), fd_mode_(fd_mode) {}

SocketCANBus::~SocketCANBus() {
  Shutdown();  // 先停采样线程
  SocketCAN_Close(socket_fd_);
  socket_fd_ = -1;
}

int SocketCANBus::Init() {
  socket_fd_ = SocketCAN_Connect(can_interface_.c_str(), fd_mode_);
  if (socket_fd_ < 0) {
    return -1;
  }
  sample_thread_ = std::jthread([this](std::stop_token st) {
    while (!st.stop_requested()) {
      Sample();
    }
  });
  return socket_fd_;
}

void SocketCANBus::Shutdown() {
  sample_thread_.request_stop();
  // 等线程退出，read()有超时兜底，最坏100ms
  if (sample_thread_.joinable()) {
    sample_thread_.join();
  }
}

void SocketCANBus::SetReceiverId(uint32_t *id, uint8_t len) {
  std::lock_guard<std::mutex> lock(fb_frames_mutex_);
  for (int i = 0; i < len; ++i) {
    fb_frames_[i].master_id = id[i];
  }
  // 将接收 ID 注册为内核 CAN 过滤器，socket 只接收这些 ID 的帧
  if (socket_fd_ > 0) {
    uint32_t ids[kMAX_MOTOR_NUM];
    for (int i = 0; i < kMAX_MOTOR_NUM; ++i) {
      ids[i] = fb_frames_[i].master_id;
    }
    SocketCAN_SetFilters(socket_fd_, ids, kMAX_MOTOR_NUM);
  }
}

void SocketCANBus::Sample() {
  uint32_t id{};
  uint8_t dlc{};
  uint8_t data[8]{};
  int bytes;
  if (fd_mode_) {
    bytes = SocketCANFD_Receive(socket_fd_, &id, &dlc, data);
  } else {
    bytes = SocketCAN_Receive(socket_fd_, &id, &dlc, data);
  }
  if (bytes <= 0) {
    // 超时/错误：不碰任何缓存帧，过期由 Receive() 的时间戳判定
    return;
  }
  std::lock_guard<std::mutex> lock(fb_frames_mutex_);
  for (int i = 0; i < kMAX_MOTOR_NUM; ++i) {
    if (fb_frames_[i].master_id == id) {
      memcpy(fb_frames_[i].data, data, dlc);
      fb_frames_[i].valid = true;
      fb_frames_[i].stamp = std::chrono::steady_clock::now();
      break;
    }
  }
}

int SocketCANBus::Send(uint32_t id, const uint8_t *data, uint8_t len) {
  if (fd_mode_) {
    return SocketCANFD_Send(socket_fd_, id, len, true, data);
  } else {
    return SocketCAN_Send(socket_fd_, id, len, data);
  }
}

int SocketCANBus::Receive(uint32_t id, uint8_t *data) {
  std::lock_guard<std::mutex> lock(fb_frames_mutex_);
  const auto now = std::chrono::steady_clock::now();
  for (int i = 0; i < kMAX_MOTOR_NUM; ++i) {
    if (fb_frames_[i].master_id != id + 0x10 || !fb_frames_[i].valid) {
      continue;
    }
    if (now - fb_frames_[i].stamp > std::chrono::milliseconds(kFrameTimeoutMs)) {
      fb_frames_[i].valid = false;
      return -1;
    }
    memcpy(data, fb_frames_[i].data, 8);
    return 0;
  }
  return -1;
}
