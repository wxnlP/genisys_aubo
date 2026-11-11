#pragma once

#include "dm_motor_driver.h"
#include "gripper_ctrl_base.h"

class GeniArmGripper : public GripperCtrlBase {
 public:
  GeniArmGripper(DmMotorDriver& motor_driver) : motor_driver_(motor_driver) {}
  ~GeniArmGripper() override = default;

  void Init() override;
  void Enable() override;
  void Disable() override;
  bool ReceiveState(GripperState& state) override;
  bool SendCommand(const GripperCommand& command) override;
  bool CalibrateZero() override;

 private:
  DmMotorDriver& motor_driver_;
};
