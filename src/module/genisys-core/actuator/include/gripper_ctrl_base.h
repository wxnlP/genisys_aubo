#pragma once

class GripperCtrlBase {
 public:
  struct GripperState {
    double position{};
    double velocity{};
    double effort{};
  };
  struct GripperCommand {
    double position{};
    double velocity{};
    double effort{};
    double stiffness{};
    double damping{};
  };

  GripperCtrlBase() = default;
  virtual ~GripperCtrlBase() = default;

  virtual void Init() {}
  virtual void Enable() {}
  virtual void Disable() {}
  virtual void Shutdown() {}

  virtual bool SendCommand(const GripperCommand& command) = 0;
  virtual bool ReceiveState(GripperState& state) = 0;
  virtual bool CalibrateZero() { return false; }
};
