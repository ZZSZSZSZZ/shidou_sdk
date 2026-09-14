# ShiDou SDK

使用 Eclipse Zenoh 为底层通信协议的高性能、跨平台 C++ 机器人客户端。

## 目录结构

```
CMakeLists.txt   # 构建示例（可用 SHIDOU_BUILD_EXAMPLES=OFF 关闭）
include/         # shidou_core 头文件 + spdlog / zenoh-cpp / zenoh-c 头文件
lib/win/         # Windows（VS2022 x64 Release）：shidou_core.lib + zenoh-c 库与 cmake 配置
lib/linux/       # Linux（Ubuntu 22.04 x86_64 Release）：libshidou_core.a + zenoh-c 库与 cmake 配置
bin/win/         # zenohc.dll
examples/        # 控制示例（set_fsm / mit_control / pushrod_control / chassis_control /
                 # get_state / get_joint / trajectory）
```

## 构建

### Windows（VS2022）

```bat
cmake -S . -B build
cmake --build build --config Release
```

### Linux（Ubuntu 22.04）

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Windows 运行示例时依赖 `zenohc.dll`（构建时会自动复制到 exe 旁；但手动部署时需手动放
到 exe 同目录或加入 PATH）。

## 在你的工程中使用

```cmake
add_subdirectory(shidou_sdk)
target_link_libraries(your_app PRIVATE shidou::core)
```

C++17、第三方依赖与 zenoh 链接项随 `shidou::core` 自动传播。

```cpp
#include "shidou/robot/robot.h"

shidou::comm::ZenohConfig cfg;
cfg.robot_address = "<robot-ip>:7447";
shidou::robot::Robot robot(cfg);
// robot.Ready() / Enable() / SetMode() / SendJoint*Target() / SendBodyTarget() /
// UploadTrajectory() / GetRobotState()
```

## 注意事项

- 预编译库为 Release-only：Windows 需 VS2022（v143 工具集）；Linux 为
  Ubuntu 22.04 构建（GCC 11 / glibc 2.35）
- 版本对应 tag；Release 页提供该版本树的 tar.gz / zip 归档

## 许可

本仓库以 BSD-3-Clause 发布（见 LICENSE）。第三方依赖许可见
[licenses/](licenses/)：zenoh-c / zenoh-cpp 为 Apache-2.0 或 EPL-2.0
双许可，spdlog 为 MIT。
