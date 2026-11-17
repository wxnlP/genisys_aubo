# genisys_aubo

## 配置环境

环境要求：

- **OS**: Ubuntu 22.04/Ubuntu 24.04
- **ROS2**：Humble/Jazzy
- **编译器**: GCC 11+，Rust编译器（Zenoh）
- **构建工具**: CMake 3.24+
- **通信硬件**: SocketCAN

### 下载 aubo C++ sdk

切换到`control_module`目录：

```bash
cd src/module/control_module
```

下载 sdk 包（https://download.aubo-robotics.cn/sdk/）：

> 注意根据自己的系统选择后缀，如 x86_64、aarch64。

- x86_64

```bash
wget -O aubo_sdk.zip https://download.aubo-robotics.cn/sdk/aubo_sdk-0.27.1-rc.4-Linux_x86_64+b46f170.zip
```

- aarch64

```bash
wget -O aubo_sdk.zip https://download.aubo-robotics.cn/sdk/aubo_sdk-0.27.1-rc.4-Linux_aarch64+b46f170.zip
```

解压 sdk：

```bash
unzip aubo_sdk.zip
mv aubo_sdk-* aubo_sdk && rm -rf ./aubo_sdk.zip
```

删除软链接：

```bash
mv aubo_sdk/lib/libstdc++.so.6 aubo_sdk/lib/libstdc++.so.6.bak
```

### AimRT 依赖

- 检查 GCC 版本（GCC 11+）

> ubuntu22.04 内置的 gcc 版本是 11.4.0，因此满足系统版本需求就无需做更新。

```bash
gcc --version
```

- 更新 CMake 版本（CMake 3.24+）

> ubuntu22.04 通过 apt 安装的 CMake 版本偏低，因此需要更新。

注意根据自己的系统选择后缀，如 x86_64、aarch64。 以 aarch64 为例：

```bash
# 拉取安装脚本
wget https://cmake.org/files/v3.28/cmake-3.28.6-linux-aarch64.sh

# 添加可执行权限
chmod +x cmake-3.28.6-linux-aarch64.sh
sudo sh cmake-3.28.6-linux-aarch64.sh --prefix=/opt/cmake

# 安装
cd /opt/cmake 
sudo mv ./cmake-3.28.6-linux-aarch64/* ./
sudo rm -rf ./cmake-3.28.6-linux-aarch64

# 添加环境变量
echo 'export PATH=/opt/cmake/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

最后检查一下版本：

```bash
cmake --version
```

- 安装 rust 编译器

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libssl-dev
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

最后，**请确保已经安装 ROS2 humble/Jazzy**。

### 编译

期望目录`src`应如下：

```
.
├── control_module
│   ├── aubo_robot.cc
│   ├── aubo_robot.h
│   ├── aubo_sdk
│   ├── CMakeLists.txt
│   ├── control_module.cc
│   └── control_module.h
├── genisys-core
│   ├── actuator
│   ├── arm_control
│   └── CMakeLists.txt
├── gripper_module
│   ├── CMakeLists.txt
│   ├── gripper_module.cc
│   └── gripper_module.h
└── ros2_proxy_module
    ├── CMakeLists.txt
    ├── ros2_proxy_module.cc
    └── ros2_proxy_module.h
```

回到根目录，编译：

> 若编译失败，建议申请 8GB 虚拟内存，然后重新编译。

```bash
./build.sh
```

## 配置文件

### CMake options

一些重点使用的 CMake 选项：

| options                              | 默认值 | 含义                      | 备注                                    |
| ------------------------------------ | ------ | ------------------------- | --------------------------------------- |
| `AIMRT_BUILD_WITH_ROS2`              | ON     | 是否使用 ROS2             |                                         |
| `AIMRT_BUILD_WITH_PROTOBUF`          | ON     | 是否使用 Protobuf 库      |                                         |
| `AIMRT_BUILD_PROTOCOLS`              | ON     | 是否编译 AimRT 提供的协议 |                                         |
| `AIMRT_BUILD_NET_PLUGIN`             | ON     | 是否编译 Net 插件         |                                         |
| `AIMRT_BUILD_ZENOH_PLUGIN`           | ON     | 是否编译 Zenoh 插件       |                                         |
| `AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN` | ON     | 是否编译录播插件          | 暂未使用，可以将该选项设为 OFF 加速编译 |
| `GENISYS_BUILD_ROS2_PROXY`           | OFF    | 是否编译 ROS2 Proxy 模块  | 当你使用ROS的时候，需要将该选项设为 ON  |

### yaml cfg

yaml cfg 中有一些配置项需要了解并灵活调整：

- aubo 配置

```yaml
aubo_ip: 192.168.3.149
aubo_rpc_port: 30004
aubo_rtde_port: 30010
aubo_user: aubo
aubo_password: "123456"
request_timeout_ms: 1000
```

`aubo_ip`需要根据上位机实际值配置，ip 获取和使用有两种方式：

1. 连接上机器人 WiFi 使得电脑与机器人处于同一网段，机器人的 WiFi IP 即`aubo_ip`；
2. 使用网线连接机器人，在上位机设置静态 IP，并将电脑的有线网络设置为与机器人同网段，上位机 IP 即`aubo_ip`。（推荐）

- tcp offset

```yaml
tcp_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

顺序为`(x,y,z,rx,ry,rz)`，位置 + ZYX 欧拉角。其中，位置单位是 m，欧拉角单位是度（deg），代码中会转换为弧度下发给控制器。

- `ros2 proxy ` 配置

```yaml
ROS2ProxyModule:
  publish_executor_name: ros2_proxy_executor
  publish_frequency: 100
  bridges:
    - type: joint_state
      sub_topic: /genisys/joint_state
      pub_topic: /joint_states
      enable: true
    - type: pose_stamped
      sub_topic: /genisys/tcp_pose
      pub_topic: /tcp_pose
      enable: true
    - type: pose_stamped
      sub_topic: /genisys/tcp_pose_expected
      pub_topic: /tcp_pose_expected
      enable: false
    - type: pose_stamped
      sub_topic: /genisys/flange_pose
      pub_topic: /flange_pose
      enable: true
```

按照需要开启话题的代理，开启代理以后，所有的 RPC 和 Channel 都是原生的 ROS2 + ROS2 Message，可以直接使用 ROS2 通信。

```bash
╰─❯ ros2 topic list -t
/flange_pose [geometry_msgs/msg/PoseStamped]
/joint_states [sensor_msgs/msg/JointState]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/tcp_pose [geometry_msgs/msg/PoseStamped]

╰─❯ ros2 service list | grep genisys
/genisys/describe_parameters
/genisys/get_parameter_types
/genisys/get_parameters
/genisys/list_parameters
/genisys/set_parameters
/genisys/set_parameters_atomically
/genisys/srv/GripperForce
/genisys/srv/GripperPos
/genisys/srv/IKCheck
/genisys/srv/MoveJ
/genisys/srv/MoveJMulti
/genisys/srv/MoveJPose
/genisys/srv/MoveJPoseMulti
/genisys/srv/MoveL
/genisys/srv/MoveLMulti
```

---

`genisys_aubo`提供 4 个参考 cfg 文件：

- `genisys_aubo_cfg.yaml`：http + zenoh 通信，不含夹爪；
- `genisys_aubo_gripper_cfg.yaml`：http + zenoh 通信，含夹爪；
- `genisys_aubo_ros2.yaml`：http + zenoh + ros2 通信，不含夹爪；
- `genisys_aubo_gripper_ros2.yaml`：http + zenoh + ros2 通信，含夹爪；

> [!WARNING]
>
> 目前自带的 geniarm 夹爪的 tcp offset 还未设置，会在后续的版本更新。

## 使用

- 运行控制器

可视化启动脚本，选择需要的 yaml 文件即可：

```
./pkg_start.sh
```

- 使用 http 进行 rpc 控制

```bash
# 启动控制系统后，在另一个终端运行
python3 examples/movej_demo.py
python3 examples/movej_pose_demo.py
python3 examples/movel_demo.py
python3 examples/movej_multi_demo.py
python3 examples/movej_pose_multi_demo.py
python3 examples/movel_multi_demo.py
python3 examples/go_home.py
python3 examples/gripper_demo.py
```

