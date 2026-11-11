#pragma once

#include <memory>
#include <string>

#include "canfd_bus.h"
#include "dm_motor_driver.h"
#include "gripper_ctrl_base.h"

namespace genisys_aubo::gripper_module {

// 从 genisys ActuatorManager 的夹爪分支精简而来：只负责 CAN 总线与夹爪电机。
class GripperManager {
 public:
  explicit GripperManager(const std::string& file);

  std::shared_ptr<GripperCtrlBase> GetGripper() { return gripper_ctrl_; }

  void Shutdown();

 private:
  std::shared_ptr<SocketCANBus> can_bus_;
  std::shared_ptr<DmMotorDriver> gripper_motor_;
  std::shared_ptr<GripperCtrlBase> gripper_ctrl_;
};

}  // namespace genisys_aubo::gripper_module
