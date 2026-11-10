#include "control_module/aubo_robot.h"

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace genisys_aubo::control_module {

namespace {

std::string ErrorText(int code) {
  switch (code) {
    case 0:
      return "ok";
    case 1:
      return "AUBO_BAD_STATE: 当前安全模式不是 Normal/ReducedMode/Recovery";
    case 2:
      return "AUBO_QUEUE_FULL: 规划队列已满";
    case 3:
      return "AUBO_BUSY: 上一条指令正在执行";
    case 13:
      return "AUBO_REQUEST_IGNORE: 路径太短，无需在该点停留";
    case -1:
      return "AUBO_BAD_STATE: 机器人模式非 Running 或线程异常";
    case -4:
      return "AUBO_TIMEOUT: 接口调用超时";
    case -5:
      return "AUBO_INVL_ARGUMENT: 参数维度不足";
    case -23:
      return "IK 不收敛";
    case -24:
      return "IK 超出机器人限制";
    case -25:
      return "IK 输入配置错误";
    case -26:
      return "IK 雅可比计算失败";
    case -27:
      return "有解析解，但均不满足选解条件";
    case -28:
      return "IK 未知错误";
    default:
      return "AUBO 错误码 " + std::to_string(code);
  }
}

}  // namespace

// ------------------------------ 连接与状态 ------------------------------

bool AuboRobot::Connect(const Config& cfg) {
  fk_pos_tolerance_ = cfg.fk_pos_tolerance;
  fk_rot_tolerance_ = cfg.fk_rot_tolerance;

  rpc_client_ = std::make_shared<arcs::aubo_sdk::RpcClient>();
  rpc_client_->setRequestTimeout(cfg.request_timeout_ms);

  if (rpc_client_->connect(cfg.ip, cfg.rpc_port) != 0) {
    AIMRT_ERROR("AUBO RPC 连接失败: {}:{}", cfg.ip, cfg.rpc_port);
    return false;
  }
  if (rpc_client_->login(cfg.user, cfg.password) != 0) {
    AIMRT_ERROR("AUBO RPC 登录失败: user={}", cfg.user);
    return false;
  }

  auto names = rpc_client_->getRobotNames();
  if (names.empty()) {
    AIMRT_ERROR("AUBO 未返回任何机器人名字");
    return false;
  }
  auto robot = rpc_client_->getRobotInterface(names.front());
  motion_control_ = robot->getMotionControl();
  robot_state_ = robot->getRobotState();
  robot_algorithm_ = robot->getRobotAlgorithm();
  robot_config_ = robot->getRobotConfig();
  runtime_machine_ = rpc_client_->getRuntimeMachine();
  if (!motion_control_ || !robot_state_ || !robot_algorithm_ || !robot_config_) {
    AIMRT_ERROR("AUBO 接口获取失败");
    return false;
  }

  // TCP offset 与 GeniSys 的 tool 对齐（单位 m / rad）
  if (robot_config_->setTcpOffset(cfg.tcp_offset) != 0) {
    AIMRT_WARN("setTcpOffset 失败，继续使用控制器当前 TCP offset");
  }

  // 启动运行时线程（SDK 示例在运动前调用）
  if (runtime_machine_ && runtime_machine_->start() != 0) {
    AIMRT_WARN("RuntimeMachine start 失败");
  }

  rtde_client_ = std::make_shared<arcs::aubo_sdk::RtdeClient>();
  if (rtde_client_->connect(cfg.ip, cfg.rtde_port) != 0) {
    AIMRT_ERROR("AUBO RTDE 连接失败: {}:{}", cfg.ip, cfg.rtde_port);
    return false;
  }
  if (rtde_client_->login(cfg.user, cfg.password) != 0) {
    AIMRT_ERROR("AUBO RTDE 登录失败: user={}", cfg.user);
    return false;
  }

  const std::vector<std::string> names_list = {
      "R1_actual_q",        "R1_actual_qd",        "R1_joint_torque",
      "R1_actual_TCP_pose", "R1_actual_tool_pose", "R1_target_TCP_pose",
      "R1_target_q"};
  const int chanel = rtde_client_->setTopic(false, names_list, cfg.rtde_freq_hz, 1);
  if (chanel < 0) {
    AIMRT_ERROR("RTDE setTopic 失败: {}", chanel);
    return false;
  }
  rtde_client_->subscribe(chanel, [this](arcs::aubo_sdk::InputParser& parser) {
    RobotStateFrame frame;
    // 顺序必须与 setTopic 的 names 一致
    frame.actual_q = parser.popVectorDouble();
    frame.actual_qd = parser.popVectorDouble();
    frame.joint_torque = parser.popVectorDouble();
    frame.actual_tcp = parser.popVectorDouble();
    frame.actual_tool = parser.popVectorDouble();
    frame.target_tcp = parser.popVectorDouble();
    frame.target_q = parser.popVectorDouble();

    // 姿态只在这里换算一次，发布时直接用
    frame.actual_tcp_orientation = RotVecToQuat(frame.actual_tcp);
    frame.actual_tool_orientation = RotVecToQuat(frame.actual_tool);
    frame.target_tcp_orientation = RotVecToQuat(frame.target_tcp);

    std::lock_guard<std::mutex> lock(state_mutex_);
    frame.seq = latest_state_.seq + 1;
    latest_state_ = std::move(frame);
  });

  AIMRT_INFO("AUBO 已连接: {} (rpc {} / rtde {}, {}Hz)", cfg.ip, cfg.rpc_port, cfg.rtde_port,
             cfg.rtde_freq_hz);
  return true;
}

void AuboRobot::Disconnect() {
  if (rtde_client_) {
    rtde_client_->disconnect();
    rtde_client_.reset();
  }
  if (rpc_client_) {
    rpc_client_->logout();
    rpc_client_->disconnect();
    rpc_client_.reset();
  }
}

bool AuboRobot::GetLatestState(RobotStateFrame& out) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (latest_state_.seq == 0) {
    return false;
  }
  out = latest_state_;
  return true;
}

std::vector<double> AuboRobot::GetCurrentJoints() const {
  RobotStateFrame frame;
  if (GetLatestState(frame) && frame.actual_q.size() >= 6) {
    return std::vector<double>(frame.actual_q.begin(), frame.actual_q.begin() + 6);
  }
  return std::vector<double>(6, 0.0);
}

// ------------------------------ 位姿与逆解 ------------------------------

Eigen::Quaterniond AuboRobot::RotVecToQuat(const std::vector<double>& pose6) const {
  if (pose6.size() < 6) {
    return Eigen::Quaterniond::Identity();
  }
  const Eigen::Vector3d rotation_vector(pose6[3], pose6[4], pose6[5]);
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

IkSolution AuboRobot::SolveIk(const std::vector<double>& pose6,
                              const std::vector<double>& qnear, bool is_flange,
                              bool do_fk) const {
  IkSolution out;
  if (!robot_algorithm_ || pose6.size() < 6 || qnear.size() < 6) {
    out.status = IkStatus::InvalidInput;
    return out;
  }

  // 直接用 SDK 的选解函数
  std::tuple<std::vector<double>, int> result =
      is_flange ? robot_algorithm_->inverseToolKinematics(qnear, pose6)
                : robot_algorithm_->inverseKinematics2(qnear, pose6);

  const std::vector<double>& q = std::get<0>(result);
  const int err = std::get<1>(result);
  if (err != 0) {
    // 只区分“输入非法”和“无解”；超限等情况一律视为无解
    out.status = (err == -5) ? IkStatus::InvalidInput : IkStatus::NoSolution;
    return out;
  }
  if (q.size() < 6) {
    out.status = IkStatus::NoSolution;
    return out;
  }

  if (do_fk) {
    auto fk = robot_algorithm_->forwardKinematics(q);
    const std::vector<double>& fk_pose = std::get<0>(fk);
    if (std::get<1>(fk) != 0 || fk_pose.size() < 6) {
      out.status = IkStatus::FkVerifyWarn;
      return out;
    }

    const Eigen::Vector3d target_position(pose6[0], pose6[1], pose6[2]);
    const Eigen::Vector3d fk_position(fk_pose[0], fk_pose[1], fk_pose[2]);
    const double position_error = (target_position - fk_position).norm();
    const double rotation_error = RotVecToQuat(pose6).angularDistance(RotVecToQuat(fk_pose));

    if (position_error > fk_pos_tolerance_ || rotation_error > fk_rot_tolerance_) {
      out.status = IkStatus::FkVerifyWarn;
      return out;
    }
  }

  out.status = IkStatus::Ok;
  out.q = q;
  return out;
}

// ------------------------------ 任务跟踪 ------------------------------

TaskContext AuboRobot::SetupTask() {
  std::lock_guard<std::mutex> lock(task_mutex_);
  current_task_ = TaskContext{};
  current_task_.state = 0;
  current_task_.id = ++task_id_;  // 与 genisys 一致：本地递增
  return current_task_;
}

bool AuboRobot::UpdateTasks(TaskContext& out) {
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (mc_state_ != MotionState::Running) {
    return false;
  }

  const bool steady = robot_state_->isSteady();
  const int exec_id = motion_control_->getExecId();

  // 还在跑
  if (exec_id != -1) {
    out = current_task_;
    return true;
  }

  // 队列空但还在减速
  if (!steady) {
    out = current_task_;
    return true;
  }

  current_task_.state = 3;  // FINISHED
  current_task_.end_timestamp = std::chrono::system_clock::now().time_since_epoch();
  out = current_task_;
  current_task_ = TaskContext{};

  mc_state_ = MotionState::Idle;

  return true;
}

// ------------------------------ 运动接口 ------------------------------

MoveRsp AuboRobot::MoveJ(const std::vector<double>& joints, double v, double a) {
  if (mc_state_ != MotionState::Idle) {
    return {-1, "已有任务在执行，拒绝并发运动"};
  }
  const int ret = motion_control_->moveJoint(joints, a, v, 0.0, 0.0);
  if (ret != 0) {
    return {-1, ErrorText(ret)};
  }
  const TaskContext task = SetupTask();
  mc_state_ = MotionState::Running;
  return {0, "MoveJ accepted", task.id, task.duration_ms};
}

MoveRsp AuboRobot::MoveJPose(const std::vector<double>& tcp_pose, double v, double a) {
  if (mc_state_ != MotionState::Idle) {
    return {-1, "已有任务在执行，拒绝并发运动"};
  }
  const IkSolution ik = SolveIk(tcp_pose, GetCurrentJoints(), false, true);
  if (ik.status != IkStatus::Ok) {
    return {-1, "MoveJPose 逆解失败: status=" + std::to_string(static_cast<int>(ik.status))};
  }
  const int ret = motion_control_->moveJoint(ik.q, a, v, 0.0, 0.0);
  if (ret != 0) {
    return {-1, ErrorText(ret)};
  }
  const TaskContext task = SetupTask();
  mc_state_ = MotionState::Running;
  return {0, "MoveJPose accepted", task.id, task.duration_ms};
}

MoveRsp AuboRobot::MoveJMulti(const std::vector<std::vector<double>>& joints, double r,
                              double v, double a) {
  if (mc_state_ != MotionState::Idle) {
    return {-1, "已有任务在执行，拒绝并发运动"};
  }
  if (joints.empty()) {
    return {-1, "MoveJMulti 路径为空"};
  }
  int ret = 0;
  for (size_t i = 0; i < joints.size(); ++i) {
    const bool last = (i + 1 == joints.size());
    ret = motion_control_->moveJoint(joints[i], a, v, last ? 0.0 : r, 0.0);
    if (ret != 0) {
      break;
    }
  }
  if (ret != 0) {
    return {-1, ErrorText(ret)};
  }
  const TaskContext task = SetupTask();
  mc_state_ = MotionState::Running;
  return {0, "MoveJMulti accepted", task.id, task.duration_ms};
}

MoveRsp AuboRobot::MoveJPoseMulti(const std::vector<std::vector<double>>& tcp_poses,
                                  double r, double v, double a) {
  if (mc_state_ != MotionState::Idle) {
    return {-1, "已有任务在执行，拒绝并发运动"};
  }
  if (tcp_poses.empty()) {
    return {-1, "MoveJPoseMulti 路径为空"};
  }

  // 逐点逆解，以上一点的解为参考，保证构型连续
  std::vector<double> reference = GetCurrentJoints();
  std::vector<std::vector<double>> joint_points;
  joint_points.reserve(tcp_poses.size());
  for (size_t i = 0; i < tcp_poses.size(); ++i) {
    const IkSolution ik = SolveIk(tcp_poses[i], reference, false, true);
    if (ik.status != IkStatus::Ok) {
      return {-1, "MoveJPoseMulti 第 " + std::to_string(i) +
                      " 个点逆解失败: status=" + std::to_string(static_cast<int>(ik.status))};
    }
    joint_points.push_back(ik.q);
    reference = ik.q;
  }

  int ret = 0;
  for (size_t i = 0; i < joint_points.size(); ++i) {
    const bool last = (i + 1 == joint_points.size());
    ret = motion_control_->moveJoint(joint_points[i], a, v, last ? 0.0 : r, 0.0);
    if (ret != 0) {
      break;
    }
  }
  if (ret != 0) {
    return {-1, ErrorText(ret)};
  }
  const TaskContext task = SetupTask();
  mc_state_ = MotionState::Running;
  return {0, "MoveJPoseMulti accepted", task.id, task.duration_ms};
}

MoveRsp AuboRobot::MoveL(const std::vector<double>& tcp_pose, double v, double a) {
  if (mc_state_ != MotionState::Idle) {
    return {-1, "已有任务在执行，拒绝并发运动"};
  }
  const int ret = motion_control_->moveLine(tcp_pose, a, v, 0.0, 0.0);
  if (ret != 0) {
    return {-1, ErrorText(ret)};
  }
  const TaskContext task = SetupTask();
  mc_state_ = MotionState::Running;
  return {0, "MoveL accepted", task.id, task.duration_ms};
}

MoveRsp AuboRobot::MoveLMulti(const std::vector<std::vector<double>>& tcp_poses, double r,
                              double v, double a) {
  if (mc_state_ != MotionState::Idle) {
    return {-1, "已有任务在执行，拒绝并发运动"};
  }
  if (tcp_poses.empty()) {
    return {-1, "MoveLMulti 路径为空"};
  }
  int ret = 0;
  for (size_t i = 0; i < tcp_poses.size(); ++i) {
    const bool last = (i + 1 == tcp_poses.size());
    ret = motion_control_->moveLine(tcp_poses[i], a, v, last ? 0.0 : r, 0.0);
    if (ret != 0) {
      break;
    }
  }
  if (ret != 0) {
    return {-1, ErrorText(ret)};
  }
  const TaskContext task = SetupTask();
  mc_state_ = MotionState::Running;
  return {0, "MoveLMulti accepted", task.id, task.duration_ms};
}

}  // namespace genisys_aubo::control_module
