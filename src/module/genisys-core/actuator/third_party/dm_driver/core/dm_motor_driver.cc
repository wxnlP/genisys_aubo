#include "dm_motor_driver.h"

#include <cmath>
#include <cstring>

inline int FloatToUint(float oFloat, float fMax, float fMin, uint8_t bits) {
  float span = fMax - fMin;
  float offset = fMin;
  return static_cast<int>(std::round(
      (oFloat - offset) * static_cast<float>((0x01 << bits) - 1) / span));
}

inline float UintToFloat(int oInt, float fMax, float fMin, uint8_t bits) {
  float span = fMax - fMin;
  float offset = fMin;
  return static_cast<float>(oInt) * span /
             static_cast<float>((0x01 << bits) - 1) +
         offset;
}

DmMotorDriver::DmMotorDriver(CanFdBusBase &_canfd_bus, uint8_t _motor_id,
                             MotorModel model)
    : canfd_bus_(_canfd_bus), motor_id_(_motor_id) {
  SetupMotorParams(model);
}

DmMotorDriver::DmMotorDriver(CanFdBusBase &_canfd_bus, uint8_t _motor_id,
                             MotorType _angle_min_limit,
                             MotorType _angle_max_limit,
                             MotorModel model)
    : canfd_bus_(_canfd_bus), motor_id_(_motor_id),
      angle_min_limit_(_angle_min_limit), angle_max_limit_(_angle_max_limit) {
  SetAngleLimits(_angle_min_limit, _angle_max_limit);
  SetupMotorParams(model);
}

void DmMotorDriver::SetupMotorParams(MotorModel model) {
  switch (model) {
  case MotorModel::DM4310:
    params_range_.pos_max = 12.5;
    params_range_.vel_max = 30.0;
    params_range_.torque_max = 10.0;
    params_range_.stiffness_min = 0.0;
    params_range_.stiffness_max = 500.0;
    params_range_.damping_min = 0.0;
    params_range_.damping_max = 5.0;
    break;
  case MotorModel::DM4340P:
    params_range_.pos_max = 12.5;
    params_range_.vel_max = 10.0;
    params_range_.torque_max = 28.0;
    params_range_.stiffness_min = 0.0;
    params_range_.stiffness_max = 500.0;
    params_range_.damping_min = 0.0;
    params_range_.damping_max = 5.0;
    break;
  default:
    break;
  }
}

/**
 * @brief 读寄存器
 *
 * @param [in] reg 寄存器
 * @return 成功返回0，否则返回1
 */
int DmMotorDriver::ReadRegister(uint8_t reg) {
  uint8_t data[4];

  data[0] = motor_id_;
  data[1] = motor_id_ >> 8;
  data[2] = 0x33;
  data[3] = reg;

  return canfd_bus_.Send(0x7FF, data, 4);
}

/**
 * @brief 写寄存器
 * @note
 * 写寄存器数据的命令会立即生效，但无法进行存储，掉电后会丢失，需要发送存储参数的指令
 * @param reg 寄存器
 * @param data 写入的数据
 * @return 成功返回0，否则返回1
 */
int DmMotorDriver::WriteRegister(uint8_t reg, const uint8_t *data) {
  uint8_t frame[8];

  frame[0] = motor_id_;
  frame[1] = motor_id_ >> 8;
  frame[2] = 0x55;
  frame[3] = reg;
  // Low byte first, high byte last
  for (int i = 0; i < 4; i++) {
    frame[4 + i] = *(data + i);
  }

  return canfd_bus_.Send(0x7FF, frame, 8);
}

/**
 * @brief 保存寄存器
 * @note 1.存储参数只在失能模式下生效
 *       2.储参数时将一次性保留全部参数
 *       3.该操作将参数写入片内flash中，每次操作时间最大为30ms，请注意留足足够的时间
 *       4.flash擦写次数约1万次，请不要频繁发送“存储参数”指令
 * @param [in] reg 寄存器
 * @return 成功返回0，否则返回1
 */
int DmMotorDriver::SaveRegister(uint8_t reg) {
  uint8_t data[4];

  data[0] = motor_id_;
  data[1] = motor_id_ >> 8;
  data[2] = 0xAA;
  data[3] = reg;

  return canfd_bus_.Send(0x7FF, data, 4);
}

int DmMotorDriver::Enable(bool enable) {
  uint16_t id = motor_id_ + ((static_cast<uint16_t>(current_mode_) - 1) << 16);
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
  if (enable) {
    data[7] = 0xFC;
    return canfd_bus_.Send(id, data, 8);
  }
  data[7] = 0xFD;
  return canfd_bus_.Send(id, data, 8);
}

int DmMotorDriver::SetMotorID(uint32_t id, bool save) {
  uint8_t data[4];
  for (int i = 0; i < 4; i++) {
    data[i] = static_cast<uint8_t>(id >> (i * 8));
  }
  if (WriteRegister(0x08, data) == -1) {
    return -1;
  }
  if (save) {
    return SaveRegister(0x08);
  }
  return 0;
}

int DmMotorDriver::SetZeroPos() {
  uint16_t id = motor_id_ + ((static_cast<uint16_t>(current_mode_) - 1) << 16);
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

  return canfd_bus_.Send(id, data, 8);
}

int DmMotorDriver::MITCtrl(MotorType pos, MotorType vel, MotorType torque,
                           MotorType stiffness, MotorType damping) {
  uint8_t data[8];
  uint16_t id = motor_id_;
  int posInt =
      FloatToUint(pos, params_range_.pos_max, -params_range_.pos_max, 16);
  int velInt =
      FloatToUint(vel, params_range_.vel_max, -params_range_.vel_max, 12);
  int torInt = FloatToUint(torque, params_range_.torque_max,
                           -params_range_.torque_max, 12);
  int kpInt = FloatToUint(stiffness, params_range_.stiffness_max,
                          params_range_.stiffness_min, 12);
  int kdInt = FloatToUint(damping, params_range_.damping_max,
                          params_range_.damping_min, 12);

  data[0] = posInt >> 8;
  data[1] = posInt;
  data[2] = velInt >> 4;
  data[3] = (velInt & 0x0F) << 4 | kpInt >> 8;
  data[4] = kpInt;
  data[5] = kdInt >> 4;
  data[6] = kdInt << 4 | torInt >> 8;
  data[7] = torInt;
  return canfd_bus_.Send(id, data, 8);
}

int DmMotorDriver::PosCtrl(MotorType pos, MotorType vel) {
  uint8_t data[8];
  uint16_t id = motor_id_ + 0x100;
  const float pos_float = static_cast<float>(pos);
  const float vel_float = static_cast<float>(vel);
  memcpy(data, &pos_float, sizeof(float));
  memcpy(data + 4, &vel_float, sizeof(float));
  return canfd_bus_.Send(id, data, 8);
}

int DmMotorDriver::VelCtrl(MotorType vel) {
  uint8_t data[8];
  uint16_t id = motor_id_ + 0x200;
  const float vel_float = static_cast<float>(vel);
  memcpy(data, &vel_float, sizeof(float));
  return canfd_bus_.Send(id, data, 4);
}

int DmMotorDriver::SetCtrlMode(CtrlMode mode, bool save) {
  uint8_t data[4]{};
  data[0] = static_cast<uint8_t>(mode);
  // write
  if (WriteRegister(0x0A, data) == -1) {
    return -1;
  }
  // save to register
  if (save) {
    if (SaveRegister(0x0A) == -1) {
      return -1;
    }
  }
  current_mode_ = mode;
  return 0;
}

int DmMotorDriver::ClearError(CtrlMode mode) {
  uint16_t id = motor_id_ + ((static_cast<uint16_t>(mode) - 1) << 16);
  uint8_t data[8];
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFB;

  return canfd_bus_.Send(id, data, 8);
}

int DmMotorDriver::GetLatestState(MotorState &state) {
  uint8_t data[8];

  if (canfd_bus_.Receive(motor_id_, data) == -1) {
    return -1;
  }

  state.id = data[0] & 0x0F;
  state.state = data[0] >> 4;
  int posInt = (data[1] << 8) | data[2];
  int velInt = (data[3] << 4) | (data[4] >> 4);
  int torqueInt = ((data[4] & 0x0F) << 8) | data[5];
  state.pos =
      UintToFloat(posInt, params_range_.pos_max, -params_range_.pos_max, 16);
  state.vel =
      UintToFloat(velInt, params_range_.vel_max, -params_range_.vel_max, 12);
  state.torque = UintToFloat(torqueInt, params_range_.torque_max,
                             -params_range_.torque_max, 12);
  return 0;
}

int DmMotorDriver::RequestState() {
  uint8_t data[4];
  data[0] = motor_id_ & 0x0F;
  data[1] = motor_id_ >> 8;
  data[2] = 0xCC;
  data[3] = 0x00;
  return canfd_bus_.Send(0x7FF, data, 4);
}

int DmMotorDriver::MoveCtrl(const CtrlCmd &cmd) {
  CtrlCmd limited_cmd = cmd;
  if (limited_cmd.pos > angle_max_limit_) {
    limited_cmd.pos = angle_max_limit_;
  }
  if (limited_cmd.pos < angle_min_limit_) {
    limited_cmd.pos = angle_min_limit_;
  }
  switch (current_mode_) {
  case CtrlMode::MIT:
    return MITCtrl(limited_cmd.pos, limited_cmd.vel, limited_cmd.torque, limited_cmd.stiffness, limited_cmd.damping);
  case CtrlMode::POS:
    return PosCtrl(limited_cmd.pos, limited_cmd.vel);
  case CtrlMode::VEL:
    return VelCtrl(limited_cmd.vel);
  default:
    return -1;
  }
}

void DmMotorDriver::SetAngleLimits(MotorType min_limit, MotorType max_limit) {
  angle_min_limit_ = min_limit;
  angle_max_limit_ = max_limit;
}

void DmMotorDriver::SetParamsRange(const MotorParamsRange &range) {
  params_range_ = range;
}