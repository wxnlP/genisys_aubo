#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "gripper_module/gripper_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>>
    aimrt_module_register_array[]{
        {"GripperModule",
         []() -> aimrt::ModuleBase* {
           return new genisys_aubo::gripper_module::GripperModule();
         }},
    };

AIMRT_PKG_MAIN(aimrt_module_register_array)
