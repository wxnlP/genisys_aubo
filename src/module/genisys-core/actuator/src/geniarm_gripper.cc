#include "geniarm_gripper.h"
#include <chrono>
#include <cmath>
#include <thread>

namespace {

// 标零验证参数
constexpr double kZeroPosTol = 0.001;  // rad，位置容差
constexpr int kZeroTimeoutMs = 800;   // 验证超时
constexpr int kZeroPollMs = 20;       // 轮询间隔

}  // namespace

void GeniArmGripper::Init() {
}

void GeniArmGripper::Enable() {
  motor_driver_.Enable(true);
}

void GeniArmGripper::Disable() {
  motor_driver_.Enable(false);
}


bool GeniArmGripper::SendCommand(const GripperCommand& command) {
  DmMotorDriver::CtrlCmd cmd;
  cmd.pos = command.position;
  cmd.vel = command.velocity;
  cmd.torque = command.effort;
  cmd.stiffness = command.stiffness;
  cmd.damping = command.damping;
  if (motor_driver_.MoveCtrl(cmd) == -1) {
    return false;
  }
  return true;
}

bool GeniArmGripper::ReceiveState(GripperState& state) {
  static auto last_request_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (now - last_request_time > std::chrono::milliseconds(100)) {
    motor_driver_.RequestState();
    last_request_time = now;
  }

  DmMotorDriver::MotorState motor_state;
  if (motor_driver_.GetLatestState(motor_state) != 0) {
    return false;
  }
  state.position = motor_state.pos;
  state.velocity = motor_state.vel;
  state.effort = motor_state.torque;
  return true;
}

bool GeniArmGripper::CalibrateZero() {
  // set zero position
  motor_driver_.SetZeroPos();

  // verify zero position
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kZeroTimeoutMs);
  while (std::chrono::steady_clock::now() < deadline) {

    motor_driver_.RequestState();
    std::this_thread::sleep_for(std::chrono::milliseconds(kZeroPollMs));  // 等反馈

    DmMotorDriver::MotorState state;
    if (motor_driver_.GetLatestState(state) == 0 && std::fabs(state.pos) <= kZeroPosTol) {
      return true;
    }
  }
  return false;
}
