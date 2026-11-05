# ShiDou SDK

使用 Eclipse Zenoh 为底层通信协议的高性能、跨平台 C++ 机器人客户端。

## 目录结构

```
GETTING_STARTED.md # 快速上手：按顺序跑通七个示例
CMakeLists.txt   # 构建示例（可用 SHIDOU_BUILD_EXAMPLES=OFF 关闭）
include/         # shidou_core 头文件 + spdlog / zenoh-cpp / zenoh-c 头文件
lib/win/         # Windows（VS2022 x64 Release）：shidou_core.lib + zenoh-c 库与 cmake 配置
lib/win-arm64/   # Windows（VS2026 ARM64 Release）：同上，arm64 版本
lib/linux/       # Linux（Ubuntu 22.04 x86_64 Release）：libshidou_core.a + zenoh-c 库与 cmake 配置
lib/linux-arm64/ # Linux（Ubuntu 22.04 aarch64 Release）：同上，aarch64 版本
bin/win/         # zenohc.dll（x64）
bin/win-arm64/   # zenohc.dll（arm64）
examples/        # 控制示例（set_fsm / mit_control / pp_control / pushrod_control /
                 # chassis_control / get_state / get_joint）
```

## 构建示例

### Windows x64（VS2022）

```bat
cmake -S . -B build
cmake --build build --config Release
```

### Windows ARM64（VS2026）

```bat
:: 先进入 ARM64 本机工具命令提示符，或先执行
:: "<VS 安装目录>\VC\Auxiliary\Build\vcvarsall.bat" arm64
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

ARM64 载荷由 VS2026 工具集构建；该基线下 CMake 没有 Visual Studio 2026 生成器，
因此用 Ninja 单配置构建。

### Linux（Ubuntu 22.04）

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

平台选择：CMake 按 `CMAKE_SYSTEM_PROCESSOR` 自动选取本平台的 `lib/<平台>/` 与
`bin/<平台>/`（Windows x64 → `lib/win/`、`bin/win/`；Windows ARM64 →
`lib/win-arm64/`、`bin/win-arm64/`；Linux x86_64 / aarch64 → `lib/linux/` 或
`lib/linux-arm64/`），命令行相同；目标处理器不在支持列表内时配置阶段即报错。在
x86_64 主机上为 aarch64 交叉构建需自备交叉工具链（本仓不含工具链文件）。

Windows 运行示例时依赖 `zenohc.dll`（构建时会自动复制到 exe 旁；但手动部署时需手动放
到 exe 同目录或加入 PATH）。

## 运行示例

可执行文件在构建目录下（Windows x64：`build/Release/`；Windows ARM64 与 Linux：
`build/`）。运行前机器人侧 zenoh 桥须已在监听，且 namespace 与桥配置一致。先用只读的
`get_state` 确认连通：

```bash
./get_state 192.168.168.168:7447
```

它只查询状态、不下发任何目标；`get_joint` 也只读关节反馈。第一次上手建议按这个顺序，
前两个不会让机器人产生运动：

1. `get_state`：只查询状态，确认地址与 namespace；
2. `set_fsm`：看清 STOP 与 ENABLED 的区别，切换本身不产生运动；
3. `pp_control`：第一次运动，每条臂的末端电机走 `+0.3 rad` 再回到原位；
4. `mit_control`（仅灵足臂）/ `pushrod_control` / `chassis_control`：按机型选一个。

每一步的讲解、输出对照与常见问题见 [GETTING_STARTED.md](GETTING_STARTED.md)。

共同约定：

- 示例接受两个可选参数：`[<ip>:<port>]` 与 `[namespace]`；示例自身的参数（如
  `pushrod_control` 的 `[target_mm]`）在前。省略时使用缺省值。
- 出厂默认 `robot_address` 为 `192.168.168.168:7447`，出厂默认 namespace 为 `robot168`。namespace 与桥
  不一致时示例收不到数据；桥未启用 namespace 时传空串：`set_fsm <ip>:<port> ""`。
- `<ip>:<port>` 为机器人侧 zenoh 桥的监听地址与端口。
- 示例不带 `--help`，只演示功能本身；参数顺序以上表与源码为准。
- 运动示例运行即下发目标，不做回车确认；运行前请确保机器人周围安全。

| 示例 | 作用 | 用法 |
| --- | --- | --- |
| `get_state` | 一次取回机器人状态并逐字段打印 | `get_state [<ip>:<port>] [namespace]` |
| `get_joint` | 每 500 ms 打印一帧关节反馈，共 10 帧、5 秒；逐电机打印位置、速度、力矩、使能、在线、报警码与温度 | `get_joint [<ip>:<port>] [namespace]` |
| `set_fsm` | 打印当前状态，演示 Enable 与 Stop | `set_fsm [<ip>:<port>] [namespace]` |
| `mit_control` | 每臂末端电机做 MIT 位置来回：当前位置 → `+0.3 rad` → 原位（各 1 秒，100 Hz） | `mit_control [<ip>:<port>] [namespace]` |
| `pp_control` | 每臂末端电机做 PP（轮廓位置）来回：当前位置 → `+0.3 rad` → 原位（每程只发一帧终点位置，帧里带速度上限与加速度） | `pp_control [<ip>:<port>] [namespace]` |
| `pushrod_control` | 使能模式下把顶杆送到目标位置（mm），随后打印 5 秒位置反馈 | `pushrod_control [target_mm] [<ip>:<port>] [namespace]` |
| `chassis_control` | 轮组正转 2 秒 → 停 1.5 秒 → 反转 2 秒，结束后切回 ENABLED | `chassis_control [<ip>:<port>] [namespace]` |

补充说明：

- `pushrod_control` 的 `[target_mm]` 缺省 0（即零点）；目标只下发一帧，机器人端按
  20 mm/s、50 mm/s² 的轮廓执行。示例不对到位做判定。
- `chassis_control` 在段内按固定周期持续下发轮速：机器人侧 500 ms 收不到新消息就把轮速归零，
  发一帧就等不会产生运动。
- 关节目标只在对应的模式态（MIT / CSP / PP）生效，目标不会切换模式；模式切换只能经
  机器人端的 `run_command` 服务，且进入模式态只有 `SetMode(mode)` 一条路：它排队到当前
  运动停稳后执行，执行时机器人先停驱动、写电机模式、再重新使能，机械臂短暂失去力矩。
  `Enable(mode)` 不是进模式态的捷径——落点恒为 ENABLED，词元选的是停在原地时跑的控制律
  （mit 速度阻尼，csp / pp 由驱动位置闭环按实测位置保持）；先 `Enable(mode)` 再
  `SetMode(mode)` 时切换只剩状态迁移，不走那段失能序列。STOP 下 `set_mode` 回 `busy`，
  所以从 STOP 或 ENABLED 进模式态是两步：先 `Enable()`（可带同一个词元），再
  `SetMode(mode)`。能力按臂型裁剪：
  灵足支持 mit / csp / pp，EYOU 支持 csp / pp，不支持的模式会被拒绝（`unsupported_mode`，
  `SetMode` 与带同名词元的 `Enable` 都过这道门）。
  `pp_control` 用的 PP 两族臂都支持（`mit_control` 只适用于灵足臂）。
- `mit_control` / `pp_control` / `pushrod_control` / `chassis_control` 会下发模式切换与运动目标，
  运行前请确保机器人周围安全。

## 在你的工程中使用

```cmake
add_subdirectory(shidou_sdk)
target_link_libraries(your_app PRIVATE shidou::core)
```

C++17、第三方依赖与 zenoh 链接项随 `shidou::core` 自动传播。

```cpp
#include "shidou/robot/robot.h"

// 地址与 namespace 是标识一台机器人的那一对；句柄自开会话，随作用域关闭。
shidou::robot::Robot robot("<robot-ip>:7447", "robot168");
// robot.Ready() / Enable() / Enable(mode) / SetMode() / SendJoint*Target() /
// SendBodyTarget() / GetRobotState()
```

## 注意事项

- 关节目标结构体带 `sequence`（目标序号）：`Send*Target` 在调用方留 0 时自动逐帧编号，
  机器人据此丢弃乱序迟到的旧帧；自行填非零值时原样发出——该值必须逐帧递增，且一个话题
  只应有一个发布者。自己构造帧时必须包含这 4 字节（目标帧比 3.5.1 及更早的机器人多出
  `sequence`，两端不能混用同一目标话题）
- 预编译库为 Release-only，由 GitHub Actions 流水线构建，Windows 两个基线与 Linux
  分列：Windows x64 为 VS2022（v143 工具集）；Windows ARM64 为 VS2026（MSVC 14.51
  工具集——该 arm64 镜像不提供 VS2022，无法与 x64 侧对齐）；Linux 两个架构
  （x86_64 / aarch64）为 Ubuntu 22.04（GCC 11 / glibc 2.35）
- 一架构一目录：Windows x64 用 `lib/win/` 与 `bin/win/`，Windows ARM64 用
  `lib/win-arm64/` 与 `bin/win-arm64/`，Linux 两个架构用 `lib/linux/` 与
  `lib/linux-arm64/`，跨架构混用会在链接期失败
- 版本对应 tag；Release 页提供该版本树的 `shidou-sdk-cpp-<version>.tar.gz` 归档，
  以及 `.sha256` / `.sha512` 校验和文件

## 许可

本仓库以 BSD-3-Clause 发布（见 LICENSE）。第三方依赖许可见
[licenses/](licenses/)：zenoh-c / zenoh-cpp 为 Apache-2.0 或 EPL-2.0
双许可，spdlog 为 MIT。
