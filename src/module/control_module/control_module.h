#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/logger/logger.h"
#include "aimrt_module_cpp_interface/module_base.h"

#include "GripperForce.aimrt_rpc.srv.h"
#include "GripperPos.aimrt_rpc.srv.h"
#include "IKCheck.aimrt_rpc.srv.h"
#include "MoveJ.aimrt_rpc.srv.h"
#include "MoveJPose.aimrt_rpc.srv.h"
#include "MoveJPoseMulti.aimrt_rpc.srv.h"
#include "MoveJMulti.aimrt_rpc.srv.h"
#include "MoveL.aimrt_rpc.srv.h"
#include "MoveLMulti.aimrt_rpc.srv.h"

#include "genisys_msgs/msg/task_context.hpp"
#include "joint_command.pb.h"
#include "joint_state.pb.h"
#include "pose.pb.h"

#include "control_module/aubo_robot.h"

using JointStateArray = aimrt::protocols::sensor::JointStateArray;
using JointCommandArray = aimrt::protocols::sensor::JointCommandArray;
using Pose = genisys::protocols::PoseStamped;
using TaskContextMsg = genisys_msgs::msg::TaskContext;

namespace genisys_aubo::control_module {

class ControlModule : public aimrt::ModuleBase {
 public:
  ControlModule() = default;
  ~ControlModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ControlModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  void StatePublishLoop();
  void TaskPublishLoop();

  void HandleMoveJ(const genisys_msgs::srv::MoveJ_Request& req,
                   genisys_msgs::srv::MoveJ_Response& rsp);
  void HandleMoveJPose(const genisys_msgs::srv::MoveJPose_Request& req,
                       genisys_msgs::srv::MoveJPose_Response& rsp);
  void HandleMoveJMulti(const genisys_msgs::srv::MoveJMulti_Request& req,
                        genisys_msgs::srv::MoveJMulti_Response& rsp);
  void HandleMoveJPoseMulti(const genisys_msgs::srv::MoveJPoseMulti_Request& req,
                            genisys_msgs::srv::MoveJPoseMulti_Response& rsp);
  void HandleMoveL(const genisys_msgs::srv::MoveL_Request& req,
                   genisys_msgs::srv::MoveL_Response& rsp);
  void HandleMoveLMulti(const genisys_msgs::srv::MoveLMulti_Request& req,
                        genisys_msgs::srv::MoveLMulti_Response& rsp);
  void HandleIKCheck(const genisys_msgs::srv::IKCheck_Request& req,
                     genisys_msgs::srv::IKCheck_Response& rsp);

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;

  AuboRobot robot_;

  std::vector<std::string> joint_names_;
  double state_publish_frequency_{200.0};
  double task_publish_frequency_{50.0};

  std::string topic_pub_joint_cmd_;
  std::string topic_pub_joint_state_;
  std::string topic_pub_tcp_pose_;
  std::string topic_pub_tcp_pose_expected_;
  std::string topic_pub_flange_pose_;
  std::string topic_pub_task_context_;
  std::string state_executor_name_;
  std::string task_executor_name_;

  aimrt::executor::ExecutorRef state_executor_;
  aimrt::executor::ExecutorRef task_executor_;

  aimrt::context::res::Publisher<JointCommandArray> pub_joint_cmd_;
  aimrt::context::res::Publisher<JointStateArray> pub_joint_state_;
  aimrt::context::res::Publisher<Pose> pub_tcp_pose_;
  aimrt::context::res::Publisher<Pose> pub_tcp_pose_expected_;
  aimrt::context::res::Publisher<Pose> pub_flange_pose_;
  aimrt::context::res::Publisher<TaskContextMsg> pub_task_context_;
};

}  // namespace genisys_aubo::control_module
