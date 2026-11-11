#include "gripper_manager.h"

#include <cstdint>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "geniarm_gripper.h"

namespace genisys_aubo::gripper_module {

GripperManager::GripperManager(const std::string& file) {
  std::string config_file_path = "./cfg/" + file;
  YAML::Node config = YAML::LoadFile(config_file_path);

  auto gripper_cfg = config["gripper"];
  if (!gripper_cfg) {
    throw std::runtime_error("gripper section missing in " + config_file_path);
  }

  std::string gripper_name = gripper_cfg["name"].as<std::string>();
  if (gripper_name != "geniarm_gripper") {
    throw std::runtime_error("Unsupported gripper: " + gripper_name);
  }

  std::string gripper_port = gripper_cfg["protocol"]["port"].as<std::string>();
  can_bus_ = std::make_shared<SocketCANBus>(gripper_port, true);
  if (can_bus_->Init() < 0) {
    throw std::runtime_error("Open erro on CAN port: " + gripper_port);
  }

  std::string gripper_model_str = gripper_cfg["model"].as<std::string>();
  DmMotorDriver::MotorModel gripper_motor_model;
  if (gripper_model_str == "DM4340P") {
    gripper_motor_model = DmMotorDriver::MotorModel::DM4340P;
  } else if (gripper_model_str == "DM4310") {
    gripper_motor_model = DmMotorDriver::MotorModel::DM4310;
  } else {
    throw std::runtime_error("Unsupported gripper model: " + gripper_model_str);
  }

  uint32_t gripper_motor_id = gripper_cfg["motor_id"].as<uint32_t>();
  uint32_t gripper_feedback_id = gripper_cfg["feedback_id"].as<uint32_t>();
  if (gripper_feedback_id != gripper_motor_id + 0x10) {
    throw std::runtime_error("gripper feedback_id 必须等于 motor_id + 0x10（当前 " +
                             std::to_string(gripper_feedback_id) + "）");
  }

  gripper_motor_ =
      std::make_shared<DmMotorDriver>(*can_bus_, gripper_motor_id, gripper_motor_model);
  gripper_ctrl_ = std::make_shared<GeniArmGripper>(*gripper_motor_);
  gripper_ctrl_->Init();

  can_bus_->SetReceiverId(&gripper_feedback_id, 1);
}

void GripperManager::Shutdown() {
  if (can_bus_) {
    can_bus_->Shutdown();
  }
}

}  // namespace genisys_aubo::gripper_module
