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
                 # get_state / get_joint）
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

## 运行示例

可执行文件在构建目录下（Windows：`build/Release/`；Linux：`build/examples/`）。运行前机器人侧
zenoh 桥须已在监听，且 namespace 与桥配置一致。

共同约定：

- 参数顺序固定：示例自身的参数在前，`[<ip>:<port>]` 与 `[namespace]` 在后，两者均可省略。
- 缺省 `robot_address` 为 `192.168.168.168:7447`，缺省 namespace 为 `robot168`。namespace 与桥
  不一致时示例等不到样本并报 `[FAIL]`；桥未启用 namespace 时传空串：
  `set_fsm <ip>:<port> ""`。
- `<ip>:<port>` 为机器人侧 zenoh 桥的监听地址与端口。
- 输出行前缀统一：`[PASS]` 为判定通过，`[FAIL]` 为判定失败，另有 `[WARN]` / `[INFO]` 提示；
  退出码 0 表示示例的所有判定均通过。

| 示例 | 作用 | 用法 | 回车确认 | 预期输出 |
| --- | --- | --- | --- | --- |
| `get_state` | 一次取回机器人状态并逐字段打印 | `get_state [<ip>:<port>] [namespace]` | 否 | 状态字段与各电机位置，末行 `[PASS] get_state completed` |
| `get_joint` | 订阅关节遥测，每 500 ms 打印一帧快照 | `get_joint [seconds] [<ip>:<port>] [namespace]` | 否 | 周期快照与速率统计，末行 `[PASS] get_joint completed`；样本断流时快照带 STALE 并出现 `[WARN]` |
| `set_fsm` | 在 STOP 与 ENABLED 之间切换 | `set_fsm [<ip>:<port>] [namespace]` | 每次回车切换一次，Ctrl+C 退出 | 每次切换打印 `[PASS] <原状态> -> fsm_state=<新状态>` |
| `mit_control` | 对每臂末端电机做 MIT 位置扫动：查询状态 → 必要时 Enable → `SetMode(POSITION)` → 以 100 Hz 下发 → 切回 ENABLED | `mit_control [<ip>:<port>] [namespace]` | 否 | 各段位置/速度打印，末行 `[PASS] Enable -> fsm_state=ENABLED` |
| `pushrod_control` | 使能模式下把顶杆送到目标位置（mm），并按反馈判定到位 | `pushrod_control [target_mm] [<ip>:<port>] [namespace]` | 是（先打印计划） | `[PASS] pushrod <起点> mm -> <目标> mm (measured <实测> mm)` |
| `chassis_control` | 驱动轮组正转 → 停 → 反转，每段按容差判定，末尾停发一段验证 500 ms 死区 | `chassis_control [<ip>:<port>] [namespace]` | 是（先打印计划） | 各段轮速打印，结束切回 ENABLED 打印 `[PASS] Enable -> fsm_state=ENABLED`；轮速未达容差时追加 `[FAIL] wheel velocities off target` |

补充说明：

- `get_joint` 的 `[seconds]` 为 0 或负数表示一直运行到进程被杀，适合在机器人旁长期挂监控。
- `pushrod_control` 的 `[target_mm]` 缺省 0（即零点）；顶杆反馈随 `joint_states` 到达，示例按
  `motor_id` 从中取顶杆位置，因此顶杆未上线时会报 `[FAIL] no position for pushrod motor ...`。
- `chassis_control` 在段内按固定周期持续下发轮速：机器人侧 500 ms 收不到新消息就把轮速归零，
  发一帧就等不会产生运动。
- 需要回车确认的两个示例在动作前打印动作计划；stdin 关闭或 Ctrl+C 视为放弃，不发送任何目标。
- `mit_control` / `pushrod_control` / `chassis_control` 会下发模式切换与运动目标，确认计划前请
  确保机器人周围安全。

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
