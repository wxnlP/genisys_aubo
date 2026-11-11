#include "control_module/control_module.h"

#include <chrono>
#include <thread>

#include <Eigen/Geometry>

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "yaml-cpp/yaml.h"

namespace genisys_aubo::control_module {

namespace {

int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// geometry_msgs/Pose → AUBO [x,y,z,rx,ry,rz]
std::vector<double> PoseMsgToAubo(const geometry_msgs::msg::Pose& pose) {
  Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                          pose.orientation.z);
  quat.normalize();
  // 取短弧：q 与 -q 表示同一旋转
  if (quat.w() < 0.0) {
    quat.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle_axis(quat);
  const Eigen::Vector3d rotation_vector = angle_axis.axis() * angle_axis.angle();
  return {pose.position.x, pose.position.y, pose.position.z, rotation_vector.x(),
          rotation_vector.y(), rotation_vector.z()};
}

void FillPose(Pose& msg, const std::vector<double>& pose6,
              const Eigen::Quaterniond& orientation, uint64_t seq, int64_t time_ns) {
  if (pose6.size() < 6) {
    return;
  }
  auto* header = msg.mutable_header();
  header->set_time_stamp(time_ns);
  header->set_frame_id("base_link");
  header->set_sequence_num(seq);

  auto* pose = msg.mutable_pose();
  auto* position = pose->mutable_position();
  position->set_x(pose6[0]);
  position->set_y(pose6[1]);
  position->set_z(pose6[2]);

  auto* orientation_msg = pose->mutable_orientation();
  orientation_msg->set_x(orientation.x());
  orientation_msg->set_y(orientation.y());
  orientation_msg->set_z(orientation.z());
  orientation_msg->set_w(orientation.w());
}

template <typename Response>
void FillMoveResponse(const MoveRsp& outcome, Response& rsp) {
  rsp.error.code = outcome.code;
  rsp.error.msg = outcome.message;
  rsp.result.task_id = outcome.task_id;
  rsp.result.duration_ms = outcome.duration_ms;
}

}  // namespace

bool ControlModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core_);
  ctx_ptr_->LetMe();

  try {
    const std::string file_path(ctx_ptr_->GetConfigFilePath());
    if (!file_path.empty()) {
      YAML::Node cfg = YAML::LoadFile(file_path);

      AuboRobot::Config robot_cfg;
      robot_cfg.ip = cfg["aubo_ip"] ? cfg["aubo_ip"].as<std::string>() : robot_cfg.ip;
      robot_cfg.rpc_port =
          cfg["aubo_rpc_port"] ? cfg["aubo_rpc_port"].as<int>() : robot_cfg.rpc_port;
      robot_cfg.rtde_port =
          cfg["aubo_rtde_port"] ? cfg["aubo_rtde_port"].as<int>() : robot_cfg.rtde_port;
      robot_cfg.user =
          cfg["aubo_user"] ? cfg["aubo_user"].as<std::string>() : robot_cfg.user;
      robot_cfg.password =
          cfg["aubo_password"] ? cfg["aubo_password"].as<std::string>() : robot_cfg.password;
      if (cfg["request_timeout_ms"]) {
        robot_cfg.request_timeout_ms = cfg["request_timeout_ms"].as<int>();
      }
      if (cfg["rtde_frequency"]) {
        robot_cfg.rtde_freq_hz = cfg["rtde_frequency"].as<double>();
      }
      if (cfg["tcp_offset"]) {
        robot_cfg.tcp_offset = cfg["tcp_offset"].as<std::vector<double>>();
      }
      if (cfg["fk_pos_tolerance"]) {
        robot_cfg.fk_pos_tolerance = cfg["fk_pos_tolerance"].as<double>();
      }
      if (cfg["fk_rot_tolerance"]) {
        robot_cfg.fk_rot_tolerance = cfg["fk_rot_tolerance"].as<double>();
      }

      if (cfg["joint_names"]) {
        joint_names_ = cfg["joint_names"].as<std::vector<std::string>>();
      }
      if (cfg["state_publish_frequency"]) {
        state_publish_frequency_ = cfg["state_publish_frequency"].as<double>();
      }
      if (cfg["task_publish_frequency"]) {
        task_publish_frequency_ = cfg["task_publish_frequency"].as<double>();
      }
      state_executor_name_ = cfg["state_publish_executor_name"].as<std::string>();
      task_executor_name_ = cfg["task_publish_executor_name"].as<std::string>();
      topic_pub_joint_cmd_ = cfg["topic_pub_joint_cmd"].as<std::string>();
      topic_pub_joint_state_ = cfg["topic_pub_joint_state"].as<std::string>();
      topic_pub_tcp_pose_ = cfg["topic_pub_tcp_pose"].as<std::string>();
      topic_pub_tcp_pose_expected_ = cfg["topic_pub_tcp_pose_expected"].as<std::string>();
      topic_pub_flange_pose_ = cfg["topic_pub_flange_pose"].as<std::string>();
      topic_pub_task_context_ = cfg["topic_pub_task_context"].as<std::string>();

      if (cfg["gripper"]) {
        const auto gripper_cfg = cfg["gripper"];
        gripper_controller_ = std::make_unique<GripperController>(
            gripper_cfg["MIT"]["stiffness"].as<double>(),
            gripper_cfg["MIT"]["damping"].as<double>());
        gripper_controller_->SetMaxRange(gripper_cfg["max_range"].as<double>());

        GripperController::ForceControlConfig force_cfg;
        if (gripper_cfg["force_control"]) {
          const auto fc = gripper_cfg["force_control"];
          force_cfg.contact_ticks = fc["contact_ticks"].as<int>();
          force_cfg.hold_damping = fc["hold_damping"].as<double>();
          force_cfg.ramp_rate = fc["ramp_rate"].as<double>();
          force_cfg.min_position_ratio = fc["min_position_ratio"].as<double>();
          force_cfg.max_target_torque = fc["max_target_torque"].as<double>();
        }
        gripper_controller_->SetForceControlConfig(force_cfg);

        gripper_executor_name_ = cfg["gripper_cmd_executor_name"].as<std::string>();
        topic_pub_gripper_cmd_ = cfg["topic_pub_gripper_cmd"].as<std::string>();
        topic_sub_gripper_state_ = cfg["topic_sub_gripper_state"].as<std::string>();
        if (cfg["gripper_cmd_frequency"]) {
          gripper_cmd_frequency_ = cfg["gripper_cmd_frequency"].as<double>();
        }
        gripper_controller_->SetSampleFrequency(gripper_cmd_frequency_);
      }

      robot_.SetLogger(GetLogger());
      if (!robot_.Connect(robot_cfg)) {
        AIMRT_ERROR("AUBO 连接失败，模块初始化终止");
        return false;
      }
    }

    if (joint_names_.size() < 6) {
      joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    }

    state_executor_ = ctx_ptr_->CreateExecutor(state_executor_name_);
    task_executor_ = ctx_ptr_->CreateExecutor(task_executor_name_);

    if (gripper_controller_) {
      gripper_executor_ = ctx_ptr_->CreateExecutor(gripper_executor_name_);
      pub_gripper_cmd_ =
          ctx_ptr_->CreatePublisher<JointCommandArray>(topic_pub_gripper_cmd_);
      sub_gripper_state_ = ctx_ptr_->CreateSubscriber<JointStateArray>(
          topic_sub_gripper_state_, [this](std::shared_ptr<const JointStateArray> msg) {
            if (msg->joints_size() < 1) {
              return;
            }
            const auto& joint = msg->joints(0);
            gripper_controller_->SetGripperState(joint.position(), joint.velocity(),
                                                 joint.effort());
          });
    }

    pub_joint_cmd_ = ctx_ptr_->CreatePublisher<JointCommandArray>(topic_pub_joint_cmd_);
    pub_joint_state_ = ctx_ptr_->CreatePublisher<JointStateArray>(topic_pub_joint_state_);
    pub_tcp_pose_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_tcp_pose_);
    pub_tcp_pose_expected_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_tcp_pose_expected_);
    pub_flange_pose_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_flange_pose_);
    pub_task_context_ = ctx_ptr_->CreatePublisher<TaskContextMsg>(topic_pub_task_context_);

    // RPC 服务
    auto movej = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJCoServer>();
    movej->MoveJ.ServeInline([this](aimrt::rpc::ContextRef,
                                    const genisys_msgs::srv::MoveJ_Request& req,
                                    genisys_msgs::srv::MoveJ_Response& rsp) {
      HandleMoveJ(req, rsp);
      return aimrt::rpc::Status();
    });

    auto movej_pose = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJPoseCoServer>();
    movej_pose->MoveJPose.ServeInline([this](aimrt::rpc::ContextRef,
                                             const genisys_msgs::srv::MoveJPose_Request& req,
                                             genisys_msgs::srv::MoveJPose_Response& rsp) {
      HandleMoveJPose(req, rsp);
      return aimrt::rpc::Status();
    });

    auto movej_multi = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJMultiCoServer>();
    movej_multi->MoveJMulti.ServeInline([this](aimrt::rpc::ContextRef,
                                               const genisys_msgs::srv::MoveJMulti_Request& req,
                                               genisys_msgs::srv::MoveJMulti_Response& rsp) {
      HandleMoveJMulti(req, rsp);
      return aimrt::rpc::Status();
    });

    auto movej_pose_multi = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJPoseMultiCoServer>();
    movej_pose_multi->MoveJPoseMulti.ServeInline(
        [this](aimrt::rpc::ContextRef,
               const genisys_msgs::srv::MoveJPoseMulti_Request& req,
               genisys_msgs::srv::MoveJPoseMulti_Response& rsp) {
          HandleMoveJPoseMulti(req, rsp);
          return aimrt::rpc::Status();
        });

    auto movel = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveLCoServer>();
    movel->MoveL.ServeInline([this](aimrt::rpc::ContextRef,
                                    const genisys_msgs::srv::MoveL_Request& req,
                                    genisys_msgs::srv::MoveL_Response& rsp) {
      HandleMoveL(req, rsp);
      return aimrt::rpc::Status();
    });

    auto movel_multi = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveLMultiCoServer>();
    movel_multi->MoveLMulti.ServeInline([this](aimrt::rpc::ContextRef,
                                               const genisys_msgs::srv::MoveLMulti_Request& req,
                                               genisys_msgs::srv::MoveLMulti_Response& rsp) {
      HandleMoveLMulti(req, rsp);
      return aimrt::rpc::Status();
    });

    auto ik_check = ctx_ptr_->CreateServer<genisys_msgs::srv::IKCheckCoServer>();
    ik_check->IKCheck.ServeInline([this](aimrt::rpc::ContextRef,
                                         const genisys_msgs::srv::IKCheck_Request& req,
                                         genisys_msgs::srv::IKCheck_Response& rsp) {
      HandleIKCheck(req, rsp);
      return aimrt::rpc::Status();
    });

    auto gripper_pos = ctx_ptr_->CreateServer<genisys_msgs::srv::GripperPosCoServer>();
    gripper_pos->GripperPos.ServeInline([this](aimrt::rpc::ContextRef,
                                               const genisys_msgs::srv::GripperPos_Request& req,
                                               genisys_msgs::srv::GripperPos_Response& rsp) {
      HandleGripperPos(req, rsp);
      return aimrt::rpc::Status();
    });

    auto gripper_force = ctx_ptr_->CreateServer<genisys_msgs::srv::GripperForceCoServer>();
    gripper_force->GripperForce.ServeInline(
        [this](aimrt::rpc::ContextRef,
               const genisys_msgs::srv::GripperForce_Request& req,
               genisys_msgs::srv::GripperForce_Response& rsp) {
          HandleGripperForce(req, rsp);
          return aimrt::rpc::Status();
        });

  } catch (const std::exception& e) {
    AIMRT_ERROR("ControlModule 初始化失败: {}", e.what());
    return false;
  }

  AIMRT_INFO("ControlModule 初始化成功");
  return true;
}

bool ControlModule::Start() {
  state_executor_.Execute([this]() {
    ctx_ptr_->LetMe();
    StatePublishLoop();
  });
  task_executor_.Execute([this]() {
    ctx_ptr_->LetMe();
    TaskPublishLoop();
  });
  if (gripper_controller_) {
    gripper_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      GripperCmdLoop();
    });
  }
  AIMRT_INFO("ControlModule 已启动");
  return true;
}

void ControlModule::Shutdown() {
  try {
    ctx_ptr_->StopRunning();
    robot_.Disconnect();
  } catch (const std::exception& e) {
    AIMRT_ERROR("ControlModule 关闭异常: {}", e.what());
  }
}

void ControlModule::StatePublishLoop() {
  const auto period = std::chrono::nanoseconds(
      static_cast<int64_t>(1e9 / state_publish_frequency_));
  auto next = std::chrono::steady_clock::now();
  uint64_t seq = 0;

  while (ctx_ptr_->Running()) {
    RobotStateFrame frame;
    if (!robot_.GetLatestState(frame)) {
      next += period;
      std::this_thread::sleep_until(next);
      continue;
    }

    const int64_t time_ns = NowNs();
    ctx_ptr_->LetMe();

    // joint_state（6 轴）
    JointStateArray joint_state;
    auto* js_header = joint_state.mutable_header();
    js_header->set_time_stamp(time_ns);
    js_header->set_frame_id("base_link");
    js_header->set_sequence_num(seq);
    for (size_t i = 0; i < 6; ++i) {
      auto* joint = joint_state.add_joints();
      joint->set_name(joint_names_[i]);
      joint->set_position(i < frame.actual_q.size() ? frame.actual_q[i] : 0.0);
      joint->set_velocity(i < frame.actual_qd.size() ? frame.actual_qd[i] : 0.0);
      joint->set_effort(i < frame.joint_torque.size() ? frame.joint_torque[i] : 0.0);
    }
    pub_joint_state_.Publish(joint_state);

    // joint_cmd：R1_target_q 回显（K/D 无对应，填 0）
    JointCommandArray joint_cmd;
    auto* jc_header = joint_cmd.mutable_header();
    jc_header->set_time_stamp(time_ns);
    jc_header->set_frame_id("base_link");
    jc_header->set_sequence_num(seq);
    for (size_t i = 0; i < 6; ++i) {
      auto* joint = joint_cmd.add_joints();
      joint->set_name(joint_names_[i]);
      joint->set_position(i < frame.target_q.size() ? frame.target_q[i] : 0.0);
      joint->set_velocity(0.0);
      joint->set_effort(0.0);
      joint->set_stiffness(0.0);
      joint->set_damping(0.0);
    }
    pub_joint_cmd_.Publish(joint_cmd);

    Pose tcp_pose;
    FillPose(tcp_pose, frame.actual_tcp, frame.actual_tcp_orientation, seq, time_ns);
    pub_tcp_pose_.Publish(tcp_pose);

    Pose tcp_pose_expected;
    FillPose(tcp_pose_expected, frame.target_tcp, frame.target_tcp_orientation, seq,
             time_ns);
    pub_tcp_pose_expected_.Publish(tcp_pose_expected);

    Pose flange_pose;
    FillPose(flange_pose, frame.actual_tool, frame.actual_tool_orientation, seq,
             time_ns);
    pub_flange_pose_.Publish(flange_pose);

    ++seq;
    next += period;
    std::this_thread::sleep_until(next);
  }
}

void ControlModule::TaskPublishLoop() {
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / task_publish_frequency_));
  auto next = std::chrono::steady_clock::now();

  while (ctx_ptr_->Running()) {
    TaskContext task;
    if (robot_.UpdateTasks(task)) {
      TaskContextMsg msg;
      const int64_t now = NowNs();
      msg.header.stamp.sec = static_cast<int32_t>(now / 1000000000LL);
      msg.header.stamp.nanosec = static_cast<uint32_t>(now % 1000000000LL);
      msg.header.frame_id = "base_link";
      msg.id = task.id;
      msg.duration_ms = task.duration_ms;
      msg.end_timestamp.sec = static_cast<int32_t>(task.end_timestamp.count() / 1000000000LL);
      msg.end_timestamp.nanosec =
          static_cast<uint32_t>(task.end_timestamp.count() % 1000000000LL);
      msg.state = task.state;
      msg.code = task.code;
      pub_task_context_.Publish(msg);
    }

    next += period;
    std::this_thread::sleep_until(next);
  }
}

// ------------------------------ RPC 处理 ------------------------------

void ControlModule::HandleMoveJ(const genisys_msgs::srv::MoveJ_Request& req,
                                genisys_msgs::srv::MoveJ_Response& rsp) {
  const std::vector<double> joints(req.position.position.begin(), req.position.position.end());
  FillMoveResponse(robot_.MoveJ(joints, req.v, req.a), rsp);
}

void ControlModule::HandleMoveJPose(const genisys_msgs::srv::MoveJPose_Request& req,
                                    genisys_msgs::srv::MoveJPose_Response& rsp) {
  FillMoveResponse(robot_.MoveJPose(PoseMsgToAubo(req.pose), req.v, req.a), rsp);
}

void ControlModule::HandleMoveJMulti(const genisys_msgs::srv::MoveJMulti_Request& req,
                                     genisys_msgs::srv::MoveJMulti_Response& rsp) {
  std::vector<std::vector<double>> joints;
  joints.reserve(req.position.size());
  for (const auto& joint : req.position) {
    joints.emplace_back(joint.position.begin(), joint.position.end());
  }
  FillMoveResponse(robot_.MoveJMulti(joints, req.r, req.v, req.a), rsp);
}

void ControlModule::HandleMoveJPoseMulti(const genisys_msgs::srv::MoveJPoseMulti_Request& req,
                                         genisys_msgs::srv::MoveJPoseMulti_Response& rsp) {
  std::vector<std::vector<double>> tcp_poses;
  tcp_poses.reserve(req.pose.size());
  for (const auto& pose : req.pose) {
    tcp_poses.push_back(PoseMsgToAubo(pose));
  }
  FillMoveResponse(robot_.MoveJPoseMulti(tcp_poses, req.r, req.v, req.a), rsp);
}

void ControlModule::HandleMoveL(const genisys_msgs::srv::MoveL_Request& req,
                                genisys_msgs::srv::MoveL_Response& rsp) {
  FillMoveResponse(robot_.MoveL(PoseMsgToAubo(req.pose), req.v, req.a), rsp);
}

void ControlModule::HandleMoveLMulti(const genisys_msgs::srv::MoveLMulti_Request& req,
                                     genisys_msgs::srv::MoveLMulti_Response& rsp) {
  std::vector<std::vector<double>> tcp_poses;
  tcp_poses.reserve(req.pose.size());
  for (const auto& pose : req.pose) {
    tcp_poses.push_back(PoseMsgToAubo(pose));
  }
  FillMoveResponse(robot_.MoveLMulti(tcp_poses, req.r, req.v, req.a), rsp);
}

void ControlModule::HandleIKCheck(const genisys_msgs::srv::IKCheck_Request& req,
                                  genisys_msgs::srv::IKCheck_Response& rsp) {
  rsp.results.clear();
  rsp.results.reserve(req.request.size());
  const std::vector<double> qnear = robot_.GetCurrentJoints();

  for (const auto& target : req.request) {
    auto& result = rsp.results.emplace_back();
    result.id = target.id;
    result.joint.position.fill(0.0);

    const bool is_tcp = (target.frame == genisys_msgs::msg::IKRequest::FRAME_TCP);
    const bool is_flange = (target.frame == genisys_msgs::msg::IKRequest::FRAME_FLANGE);
    if (!is_tcp && !is_flange) {
      result.status = genisys_msgs::srv::IKCheck_Response::INVALID_INPUT;
      result.solved = false;
      continue;
    }

    const IkSolution ik =
        robot_.SolveIk(PoseMsgToAubo(target.pose), qnear, is_flange, target.do_fk);
    result.status = static_cast<uint8_t>(ik.status);
    result.solved = (ik.status == IkStatus::Ok);
    if (result.solved) {
      for (int i = 0; i < 6 && i < static_cast<int>(ik.q.size()); ++i) {
        result.joint.position[i] = ik.q[i];
      }
    }
  }

  rsp.error.code = 0;
  rsp.error.msg = "IKCheck succeeded";
}

void ControlModule::HandleGripperPos(const genisys_msgs::srv::GripperPos_Request& req,
                                     genisys_msgs::srv::GripperPos_Response& rsp) {
  if (!gripper_controller_) {
    AIMRT_ERROR("收到 GripperPos 请求，但当前配置未启用夹爪");
    rsp.error.code = -1;
    rsp.error.msg = "Gripper is not configured";
    return;
  }
  const bool ret = gripper_controller_->PosControl(req.position, req.velocity,
                                                   req.acceleration, req.effort);
  if (ret) {
    gripper_controller_->WaitForCompletion();
    rsp.error.code = 0;
    rsp.error.msg = "GripperPos succeeded";
  } else {
    rsp.error.code = -1;
    rsp.error.msg = "GripperPos failed";
  }
}

void ControlModule::HandleGripperForce(const genisys_msgs::srv::GripperForce_Request& req,
                                       genisys_msgs::srv::GripperForce_Response& rsp) {
  if (!gripper_controller_) {
    AIMRT_ERROR("收到 GripperForce 请求，但当前配置未启用夹爪");
    rsp.error.code = -1;
    rsp.error.msg = "Gripper is not configured";
    return;
  }

  GripperController::ForceControlParams params;
  params.approach_velocity = req.approach_velocity;
  params.approach_accel = req.approach_accel;
  params.switch_velocity = req.switch_velocity;
  params.target_torque = req.target_torque;
  params.closed_release = req.closed_release;

  const int ret = gripper_controller_->ForceControl(params);
  if (ret == 0) {
    rsp.error.code = rsp.OK;
    rsp.error.msg = "GripperForce succeeded";
  } else if (ret == -1) {
    rsp.error.code = rsp.FAILED;
    rsp.error.msg = "GripperForce failed";
  } else if (ret == -2) {
    rsp.error.code = rsp.EMPTY;
    rsp.error.msg = "GripperForce Empty (object not grasped)";
  }
}

void ControlModule::GripperCmdLoop() {
  const auto period =
      std::chrono::nanoseconds(static_cast<int64_t>(1e9 / gripper_cmd_frequency_));
  auto next = std::chrono::steady_clock::now();

  while (ctx_ptr_->Running()) {
    GripperController::GripperCommand cmd;
    if (gripper_controller_->GetGripperCommand(cmd) == -1) {
      // 无规划轨迹 — 复用上一帧指令保持
      if (last_gripper_cmd_.has_value()) {
        cmd = *last_gripper_cmd_;
      } else {
        double current_position = 0.0;
        if (gripper_controller_->GetCurrentPosition(current_position) != 0) {
          next += period;
          std::this_thread::sleep_until(next);
          continue;
        }
        cmd.position = current_position;
        cmd.velocity = 0.0;
        cmd.effort = 0.0;
        last_gripper_cmd_ = cmd;
      }
    } else {
      last_gripper_cmd_ = cmd;
    }

    JointCommandArray msg;
    auto* header = msg.mutable_header();
    header->set_time_stamp(NowNs());
    header->set_frame_id("base_link");

    auto* joint = msg.add_joints();
    joint->set_name("gripper");
    joint->set_position(cmd.position);
    joint->set_velocity(cmd.velocity);
    joint->set_effort(cmd.effort);
    joint->set_stiffness(cmd.stiffness);
    joint->set_damping(cmd.damping);

    ctx_ptr_->LetMe();
    pub_gripper_cmd_.Publish(msg);

    next += period;
    std::this_thread::sleep_until(next);
  }
}

}  // namespace genisys_aubo::control_module
