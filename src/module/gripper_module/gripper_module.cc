#include "gripper_module/gripper_module.h"

#include <chrono>
#include <thread>

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "gripper_manager.h"
#include "yaml-cpp/yaml.h"

namespace genisys_aubo::gripper_module {

bool GripperModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core_);
  ctx_ptr_->LetMe();

  try {
    const std::string file_path(ctx_ptr_->GetConfigFilePath());
    AIMRT_CHECK_ERROR_THROW(!file_path.empty(), "GripperModule config file path is empty");

    YAML::Node cfg = YAML::LoadFile(file_path);

    const auto config_file = cfg["config_file"].as<std::string>();
    topic_sub_joint_cmd_ = cfg["topic_sub_gripper_cmd"].as<std::string>();
    topic_pub_gripper_state_ = cfg["topic_pub_gripper_state"].as<std::string>();
    pub_executor_name_ = cfg["pub_executor_name"].as<std::string>();
    write_executor_name_ = cfg["write_executor_name"].as<std::string>();
    if (cfg["pub_frequency"]) {
      publish_frequency_ = cfg["pub_frequency"].as<double>();
    }
    joint_name_ = cfg["joint_name"] ? cfg["joint_name"].as<std::string>() : "gripper";

    gripper_manager_ = std::make_shared<GripperManager>(config_file);
    gripper_ = gripper_manager_->GetGripper();
    AIMRT_CHECK_ERROR_THROW(gripper_, "Failed to create gripper actuator");

    pub_executor_ = ctx_ptr_->CreateExecutor(pub_executor_name_);
    write_executor_ = ctx_ptr_->CreateExecutor(write_executor_name_);

    pub_gripper_state_ =
        ctx_ptr_->CreatePublisher<JointStateArray>(topic_pub_gripper_state_);
    sub_joint_cmd_ = ctx_ptr_->CreateSubscriber<JointCommandArray>(
        topic_sub_joint_cmd_, std::bind(&GripperModule::JointCmdCallback, this,
                                        std::placeholders::_1));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");
  return true;
}

bool GripperModule::Start() {
  try {
    gripper_->Enable();

    pub_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      PublishLoop();
    });
    write_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      WriteLoop();
    });
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void GripperModule::Shutdown() {
  try {
    ctx_ptr_->StopRunning();
    write_sem_.release();

    if (gripper_) {
      gripper_->Disable();
    }
    if (gripper_manager_) {
      gripper_manager_->Shutdown();
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("Disable gripper failed, {}", e.what());
  }
  AIMRT_INFO("Shutdown succeeded.");
}

void GripperModule::PublishLoop() {
  const auto period =
      std::chrono::nanoseconds(static_cast<int64_t>(1e9 / publish_frequency_));
  auto next = std::chrono::steady_clock::now();
  uint64_t seq = 0;

  JointStateArray msg;
  GripperCtrlBase::GripperState state;

  while (ctx_ptr_->Running()) {
    if (!gripper_->ReceiveState(state)) {
      next += period;
      std::this_thread::sleep_until(next);
      continue;
    }

    msg.Clear();
    auto* header = msg.mutable_header();
    header->set_time_stamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    header->set_frame_id("base_link");
    header->set_sequence_num(seq++);

    auto* joint = msg.add_joints();
    joint->set_name(joint_name_);
    joint->set_position(state.position);
    joint->set_velocity(state.velocity);
    joint->set_effort(state.effort);

    ctx_ptr_->LetMe();
    pub_gripper_state_.Publish(msg);

    next += period;
    std::this_thread::sleep_until(next);
  }
}

void GripperModule::WriteLoop() {
  GripperCtrlBase::GripperCommand cmd;
  while (ctx_ptr_->Running()) {
    if (!write_sem_.try_acquire_for(std::chrono::milliseconds(100))) {
      continue;
    }
    if (!ctx_ptr_->Running()) {
      break;
    }
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd = cached_cmd_;
    }
    if (!gripper_->SendCommand(cmd)) {
      AIMRT_WARN("夹爪指令下发失败");
    }
  }
}

void GripperModule::JointCmdCallback(const std::shared_ptr<const JointCommandArray>& msg) {
  if (!ctx_ptr_->Running()) {
    return;
  }
  if (msg->joints_size() < 1) {
    AIMRT_WARN("joint_cmd 关节数为 0，丢弃");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    const auto& joint = msg->joints(0);
    cached_cmd_.position = joint.position();
    cached_cmd_.velocity = joint.velocity();
    cached_cmd_.effort = joint.effort();
    cached_cmd_.stiffness = joint.stiffness();
    cached_cmd_.damping = joint.damping();
  }

  write_sem_.release();
}

}  // namespace genisys_aubo::gripper_module
