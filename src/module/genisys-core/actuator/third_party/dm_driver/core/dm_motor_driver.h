#pragma once

#include "canfd_bus_base.h"
#include <cstdint>

using MotorType = double;

class DmMotorDriver {
public:
  enum class CtrlMode {
    MIT = 1, // 混合控制（力、位、速、刚度、阻尼）
    POS,     // 位置控制
    VEL,     // 速度控制
  };

  struct MotorState {
    uint32_t id{};      // motor id
    int state{};        // motor state
    MotorType pos{};    // real position
    MotorType vel{};    // real velocity
    MotorType torque{}; // real torque
  };

  struct CtrlCmd {
    MotorType pos{};       // 位置 rad
    MotorType vel{};       // 速度 rad/s
    MotorType torque{};    // 力矩 N.M
    MotorType stiffness{}; // 刚度 N/r
    MotorType damping{};   // 阻尼 N*s/r
  };

  struct MotorParamsRange {
    MotorType pos_max{12.5};
    MotorType vel_max{30.0};
    MotorType torque_max{10.0};
    MotorType stiffness_min{0.0};
    MotorType stiffness_max{500.0};
    MotorType damping_min{0.0};
    MotorType damping_max{5.0};
  };

  enum class MotorModel {
    DM4310 = 1,
    DM4340P = 2,
  };

  DmMotorDriver(CanFdBusBase &_canfd_bus, uint8_t _motor_id,
                MotorModel model = MotorModel::DM4310);
  DmMotorDriver(CanFdBusBase &_canfd_bus, uint8_t _motor_id,
                MotorType _angle_min_limit, MotorType _angle_max_limit,
                MotorModel model = MotorModel::DM4310);
  virtual ~DmMotorDriver() = default;

  int Enable(bool enable);
  void SetParamsRange(const MotorParamsRange &range);
  int SetCtrlMode(CtrlMode mode, bool save);
  int SetZeroPos();
  int SetMotorID(uint32_t id, bool save);
  int MoveCtrl(const CtrlCmd &cmd);
  int RequestState();
  int GetLatestState(MotorState &state);
  int ClearError(CtrlMode mode);
  void SetAngleLimits(MotorType min_limit, MotorType max_limit);

private:
  void SetupMotorParams(MotorModel model);
  int MITCtrl(MotorType pos, MotorType vel, MotorType torque,
              MotorType stiffness, MotorType damping);
  int PosCtrl(MotorType pos, MotorType vel);
  int VelCtrl(MotorType vel);
  // Register access helpers.
  int ReadRegister(uint8_t reg);
  int WriteRegister(uint8_t reg, const uint8_t *data);
  int SaveRegister(uint8_t reg);

  CanFdBusBase &canfd_bus_;
  uint32_t motor_id_{0x01};
  CtrlMode current_mode_{CtrlMode::MIT};
  MotorType angle_min_limit_{-3.1415};
  MotorType angle_max_limit_{3.1415};
  MotorParamsRange params_range_{};
};
