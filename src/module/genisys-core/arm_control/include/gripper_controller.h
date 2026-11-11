#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include "spsc_ring_buffer.h"

class GripperController {
 public:
  struct GripperCommand {
    double position{};
    double velocity{};
    double effort{};
    double stiffness{};
    double damping{};
  };

  struct ForceControlParams {
    double approach_velocity{1.0};  // 接近速度 rad/s
    double approach_accel{1.0};     // 接近加速度 rad/s^2
    double switch_velocity{0.2};    // 速度亏欠阈值 rad/s
    double target_torque{0.30};     // 力矩控制值 N·m：夹持力矩
    bool closed_release{true};      // 位置接近闭合时是否释放力矩
  };

  struct ForceControlConfig {
    int contact_ticks{3};             // 接触确认帧数，防单帧电流尖峰
    double hold_damping{0.3};         // 保持段阻尼
    double ramp_rate{2.0};            // 力矩爬升率 N·m/s
    double min_position_ratio{0.05};  // 「位置接近闭合」阈值 = 比例 × max_range
    double max_target_torque{3.0};    // 力矩控制值上限 N·m
  };

  GripperController(double stiffness, double damping) : stiffness_(stiffness), damping_(damping) {}

  void SetGripperState(double position, double velocity, double effort);
  int GetCurrentPosition(double& position);
  int GetGripperCommand(GripperCommand& command);
  void SetMaxRange(double max_range) { max_range_ = max_range; }
  void SetSampleFrequency(double hz) { sample_frequency_ = hz; }
  double GetSampleFrequency() const { return sample_frequency_; }
  void WaitForCompletion();
  bool PosControl(double position, double velocity, double accel, double effort);
  int ForceControl(const ForceControlParams& params);

  void SetForceControlConfig(const ForceControlConfig& config) { force_config_ = config; }
  const ForceControlConfig& GetForceControlConfig() const { return force_config_; }

 private:
  double stiffness_{0.0};
  double damping_{0.0};
  double max_range_{0.0};
  double sample_frequency_ = 100.0;  // Hz

  // current state
  std::optional<double> current_position_{};
  std::optional<double> current_velocity_{};
  std::optional<double> current_effort_{};

  std::optional<double> last_cmd_position_{};

  std::mutex state_mutex_;
  std::mutex last_cmd_mutex_;
  SPSCRingBuffer<GripperCommand> command_queue_{4096};

  std::atomic<bool> force_stop_{false};
  ForceControlConfig force_config_{};

  double approach_torque_ff_{0.3};
};
