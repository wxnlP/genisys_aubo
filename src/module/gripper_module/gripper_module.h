#pragma once

#include <memory>
#include <mutex>
#include <semaphore>
#include <string>

#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "gripper_ctrl_base.h"
#include "joint_command.pb.h"
#include "joint_state.pb.h"

using JointStateArray = aimrt::protocols::sensor::JointStateArray;
using JointCommandArray = aimrt::protocols::sensor::JointCommandArray;

namespace genisys_aubo::gripper_module {

class GripperManager;

// 夹爪执行器模块：独占 CAN 总线，发布夹爪状态，按 joint_cmd 写电机。
class GripperModule : public aimrt::ModuleBase {
 public:
  GripperModule() = default;
  ~GripperModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "GripperModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  void PublishLoop();
  void WriteLoop();
  void JointCmdCallback(const std::shared_ptr<const JointCommandArray>& msg);

  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::context::res::Subscriber<JointCommandArray> sub_joint_cmd_;
  aimrt::context::res::Publisher<JointStateArray> pub_gripper_state_;
  aimrt::executor::ExecutorRef pub_executor_;
  aimrt::executor::ExecutorRef write_executor_;

  std::binary_semaphore write_sem_{0};
  std::mutex cmd_mutex_;
  GripperCtrlBase::GripperCommand cached_cmd_;

  std::string topic_sub_joint_cmd_;
  std::string topic_pub_gripper_state_;
  std::string pub_executor_name_;
  std::string write_executor_name_;
  std::string joint_name_;
  double publish_frequency_{200.0};

  std::shared_ptr<GripperManager> gripper_manager_;
  std::shared_ptr<GripperCtrlBase> gripper_;
};

}  // namespace genisys_aubo::gripper_module
