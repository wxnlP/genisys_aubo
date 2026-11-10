# genisys_aubo

## 配置 aubo sdk 环境

### 下载 aubo C++ sdk

切换到`src`目录：

```bash
cd src
```

下载 sdk 包：https://download.aubo-robotics.cn/sdk/

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

目录：

```
.
├── aubo_sdk
│   ├── bin
│   ├── include
│   ├── lib
│   ├── share
│   └── VERSION
├── CMakeLists.txt
├── main.cc
└── README.md
```

`CMakeLists.txt`文件：

```cmake
cmake_minimum_required(VERSION 3.28)

project(genisys_aubo_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/aubo_sdk")

find_package(aubo_sdk REQUIRED)

# 链接示例
add_executable(main main.cc)
target_link_libraries(main PRIVATE aubo_sdk::aubo_sdk)
```

编译：

```bash
cmake -S. -Bbuild
cmake --build build
```



