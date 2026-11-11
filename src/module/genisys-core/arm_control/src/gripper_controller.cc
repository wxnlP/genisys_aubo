#include "gripper_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include "arm_logger.h"
#include "lspb_time_scaler.h"
#include "quintic_time_scaler.h"

namespace {
constexpr double kBumplessTolRad = 0.05;
}  // namespace

void GripperController::SetGripperState(double position, double velocity, double effort) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_position_ = position;
  current_velocity_ = velocity;
  current_effort_ = effort;
}

int GripperController::GetCurrentPosition(double& position) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_position_.has_value()) {
    return -1;
  }
  position = *current_position_;
  return 0;
}

int GripperController::GetGripperCommand(GripperCommand& command) {
  if (command_queue_.Pop(command)) {
    {
      std::lock_guard<std::mutex> lock(last_cmd_mutex_);
      last_cmd_position_ = command.position;
    }
    return 0;
  }
  command.damping = damping_;
  command.stiffness = stiffness_;
  return -1;
}

void GripperController::WaitForCompletion() {
  while (!command_queue_.IsEmpty()) {
    std::this_thread::yield();
  }
}

bool GripperController::PosControl(double position, double velocity, double accel, double effort) {
  double current_position;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!current_position_.has_value()) {
      ARM_LOG_ERROR("[PosControl] current_position_ is not set");
      return false;
    }
    current_position = *current_position_;
  }

  double start_position = current_position;
  {
    std::lock_guard<std::mutex> lock(last_cmd_mutex_);
    if (last_cmd_position_.has_value() &&
        std::abs(*last_cmd_position_ - current_position) <= kBumplessTolRad) {
      start_position = *last_cmd_position_;
    } else if (last_cmd_position_.has_value()) {
      ARM_LOG_WARN("[Gripper] fallback to measured: delta={:.4f} rad > {:.3f} rad",
                   std::abs(*last_cmd_position_ - current_position), kBumplessTolRad);
    }
  }

  double target_pos = position * max_range_;

  double distance = target_pos - start_position;
  if (std::abs(distance) < 1e-6) {
    return true;
  }

  const double abs_dist = std::abs(distance);
  if (velocity <= 0.0 || accel <= 0.0 || abs_dist < 1e-9) {
    return false;
  }

  LSPBTimeScaler scaler;
  TimeScalerBase::PlanConfig config;
  config.v_max = velocity / abs_dist;
  config.a_max = accel / abs_dist;
  if (!scaler.Plan(config)) {
    return false;
  }

  const double duration = scaler.GetDuration();
  double t = 0.0;
  while (t <= duration + 1e-7) {
    if (t > duration) {
      t = duration;
    }

    TimeScalerBase::State state;
    if (!scaler.Evaluate(t, state)) {
      return false;
    }

    GripperCommand cmd;
    cmd.position = start_position + state.s * distance;
    cmd.velocity = state.sd * distance;
    cmd.effort = effort;
    cmd.stiffness = stiffness_;
    cmd.damping = damping_;

    if (!command_queue_.Push(cmd)) {
      ARM_LOG_ERROR("[Gripper] Command queue full! Dropping trajectory point.");
    }

    t += 1.0 / sample_frequency_;
  }

  return true;
}

int GripperController::ForceControl(const ForceControlParams& params) {
  if (params.target_torque <= 0.0 || params.target_torque > force_config_.max_target_torque ||
      params.switch_velocity <= 0.0 || params.approach_velocity <= 0.0 ||
      params.approach_accel <= 0.0) {
    ARM_LOG_ERROR(
        "[Gripper] ForceControl 参数非法: target={:.3f} (上限 {:.3f}), switch_v={:.3f}, "
        "v={:.3f}, a={:.3f}",
        params.target_torque, force_config_.max_target_torque, params.switch_velocity,
        params.approach_velocity, params.approach_accel);
    return -1;
  }

  force_stop_.store(false);
  const double dt = 1.0 / sample_frequency_;
  const auto tick = std::chrono::duration<double>(dt);
  const double release_pos = force_config_.min_position_ratio * max_range_;

  auto read_state = [this](double& x, double& v, double& tau) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!current_position_.has_value() || !current_velocity_.has_value() || !current_effort_.has_value()) {
      return false;
    }
    x = *current_position_;
    v = *current_velocity_;
    tau = *current_effort_;
    return true;
  };
  auto push = [this](double position, double velocity, double effort, double stiffness, double damping) {
    GripperCommand cmd;
    cmd.position = position;
    cmd.velocity = velocity;
    cmd.effort = effort;
    cmd.stiffness = stiffness;
    cmd.damping = damping;
    if (!command_queue_.Push(cmd)) {
      ARM_LOG_ERROR("[Gripper] 指令队列满");
    }
  };

  auto release = [&]() {
    PosControl(1.0, params.approach_velocity, params.approach_accel, 0.0);
  };

  double x = 0.0, v = 0.0, tau = 0.0;
  if (!read_state(x, v, tau)) {
    ARM_LOG_ERROR("[Gripper] 力控启动失败：读不到夹爪状态");
    return -1;
  }

  // ---------------- 第 1 步：合，直到碰到物体 ----------------
  const double x_start = x;
  const double distance = -x_start;  // 目标 0 = 完全闭合
  const double abs_dist = std::abs(distance);

  const double dir = (distance >= 0.0) ? 1.0 : -1.0;

  double duration = 0.0;
  QuinticTimeScaler scaler;
  if (abs_dist > 1e-6) {
    TimeScalerBase::PlanConfig config;
    config.v_max = params.approach_velocity / abs_dist;
    config.a_max = params.approach_accel / abs_dist;
    config.T = std::max(1.875 / config.v_max, std::sqrt(5.7735 / config.a_max));
    if (!scaler.Plan(config)) {
      ARM_LOG_ERROR("[Gripper] 接近段多项式时间标定失败");
      return -1;
    }
    duration = scaler.GetDuration();
  }

  double t = 0.0;
  int confirm = 0;
  bool contact = false;
  double pos_plan{0}, v_plan{0};
  while (read_state(x, v, tau)) {
    if (t >= duration) {
      if (params.closed_release) {
        release();
        ARM_LOG_WARN("[Gripper] 夹空（超时）");
        return -2;
      } else {
        break;
      }
    }

    double v_deficit = v_plan - v;
    if (std::abs(v_deficit) >= params.switch_velocity) {
      if (++confirm >= force_config_.contact_ticks) {
        contact = true;
        ARM_LOG_INFO("[Gripper] 接触确认，位置 {:.4f} rad, 速度 {:.4f}, 反馈力矩 {:.4f} N·m",
               x, v, tau);
        break;
      }
    } else {
      confirm = 0;
    }

    TimeScalerBase::State state{};
    scaler.Evaluate(std::min(t, duration), state);

    pos_plan = x_start + state.s * distance;
    v_plan = state.sd * distance;
    push(pos_plan, v_plan, dir * approach_torque_ff_, stiffness_, damping_);

    // 空夹
    if (x <= release_pos) {
      release();
      ARM_LOG_WARN("[Gripper] 夹空（限位）");
      return -2;
    }

    t += dt;
    std::this_thread::sleep_for(tick);
  }

  // ---------------- 第 2 步：夹住 ----------------
  double tau_cmd = std::min(std::abs(tau), params.target_torque);

  while (read_state(x, v, tau)) {
    tau_cmd = std::min(tau_cmd + force_config_.ramp_rate * dt, params.target_torque);
    push(0.0, 0.0, dir * tau_cmd, 0.0, force_config_.hold_damping);

    if (tau_cmd >= params.target_torque) {
      break;
    }
    std::this_thread::sleep_for(tick);
  }

  ARM_LOG_INFO("[Gripper] 力控完成，位置 {:.4f} rad, 反馈力矩 {:.4f} N·m", x, tau);

  return 0;
}
