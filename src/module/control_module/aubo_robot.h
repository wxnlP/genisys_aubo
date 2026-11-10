#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "aimrt_module_cpp_interface/logger/logger.h"
#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"

namespace genisys_aubo::control_module {

// RTDE 推送的一帧状态。
// 位置单位 m，角度单位 rad，关节单位 rad / rad/s / N·m。
struct RobotStateFrame {
  std::vector<double> actual_q;
  std::vector<double> actual_qd;
  std::vector<double> joint_torque;
  std::vector<double> actual_tcp;   // x,y,z,rx,ry,rz（含 TCP offset）
  std::vector<double> actual_tool;  // x,y,z,rx,ry,rz（法兰，不含 TCP）
  std::vector<double> target_tcp;   // x,y,z,rx,ry,rz
  std::vector<double> target_q;
  Eigen::Quaterniond actual_tcp_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond actual_tool_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond target_tcp_orientation{Eigen::Quaterniond::Identity()};
  uint64_t seq{0};
};

enum class MotionState {
  Idle = 0,
  Running,
};

// 与 genisys_msgs/msg/TaskContext 对齐
struct TaskContext {
  uint64_t id{0};
  double duration_ms{0.0};
  std::chrono::nanoseconds end_timestamp{0};
  int8_t state{0};
  int32_t code{0};
};

enum class IkStatus {
  Ok = 0,
  NoSolution = 1,
  InvalidInput = 2,
  FkVerifyWarn = 3,
  OtherErrors = 4,
};

struct IkSolution {
  IkStatus status{IkStatus::NoSolution};
  std::vector<double> q;  // 仅 status == IkStatus::Ok 时有效
};

// 一次运动受理的响应
struct MoveRsp {
  int code{0};  // 0 成功，-1 失败
  std::string message;
  uint64_t task_id{0};
  double duration_ms{0.0};
};

// AUBO 机器人：连接、状态、逆解、运动受理、任务跟踪
class AuboRobot {
 public:
  struct Config {
    std::string ip{};
    int rpc_port{};
    int rtde_port{};
    std::string user{};
    std::string password{};
    int request_timeout_ms{1000};
    double rtde_freq_hz{200.0};
    std::vector<double> tcp_offset{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double fk_pos_tolerance{1e-4};
    double fk_rot_tolerance{1e-4};
  };

  void SetLogger(aimrt::logger::LoggerRef logger) { logger_ = logger; }

  bool Connect(const Config& cfg);
  void Disconnect();

  bool GetLatestState(RobotStateFrame& out) const;
  std::vector<double> GetCurrentJoints() const;

  IkSolution SolveIk(const std::vector<double>& pose6, const std::vector<double>& qnear,
                     bool is_flange, bool do_fk) const;

  // 运动接口
  MoveRsp MoveJ(const std::vector<double>& joints, double v, double a);
  MoveRsp MoveJPose(const std::vector<double>& tcp_pose, double v, double a);
  MoveRsp MoveJMulti(const std::vector<std::vector<double>>& joints, double r, double v,
                     double a);
  MoveRsp MoveJPoseMulti(const std::vector<std::vector<double>>& tcp_poses, double r,
                         double v, double a);
  MoveRsp MoveL(const std::vector<double>& tcp_pose, double v, double a);
  MoveRsp MoveLMulti(const std::vector<std::vector<double>>& tcp_poses, double r, double v,
                     double a);

  bool UpdateTasks(TaskContext& out);

 private:
  auto GetLogger() const { return logger_; }

  Eigen::Quaterniond RotVecToQuat(const std::vector<double>& pose6) const;

  TaskContext SetupTask();

  aimrt::logger::LoggerRef logger_;

  arcs::aubo_sdk::RpcClientPtr rpc_client_;
  arcs::aubo_sdk::RtdeClientPtr rtde_client_;
  arcs::common_interface::MotionControlPtr motion_control_;
  arcs::common_interface::RobotStatePtr robot_state_;
  arcs::common_interface::RobotAlgorithmPtr robot_algorithm_;
  arcs::common_interface::RobotConfigPtr robot_config_;
  arcs::common_interface::RuntimeMachinePtr runtime_machine_;

  double fk_pos_tolerance_{1e-4};
  double fk_rot_tolerance_{1e-4};

  mutable std::mutex state_mutex_;
  RobotStateFrame latest_state_;

  std::mutex task_mutex_;
  uint64_t task_id_{0};
  TaskContext current_task_{};

  std::atomic<MotionState> mc_state_{MotionState::Idle};
};

}  // namespace genisys_aubo::control_module
