# genisys_aubo

## 配置 aubo sdk 环境

### 下载 aubo C++ sdk

切换到`control_module`目录：

```bash
cd src/module/control_module
```

下载 sdk 包（https://download.aubo-robotics.cn/sdk/）：

- x86-64

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

### 编译

期望目录应如下：

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

```bash
./build.sh
```


