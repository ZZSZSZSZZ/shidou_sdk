# 快速上手

本手册带你把 `examples/` 里的七个示例按「先只读、再切换、最后运动」的顺序跑一遍。
每个示例都是自包含的单个源码文件，只演示对应的 SDK 调用，没有额外的参数解析或输出约定。

## 动手之前

- 机器人已上电，机器人侧的 zenoh 桥已在监听（本仓默认地址 `192.168.168.168:7447`）。
- 上位机与机器人网络互通。
- namespace 与桥的配置一致：本仓默认 `robot168`；桥没有启用 namespace 时传空串 `""`。

所有示例接受两个可选参数：`[<ip>:<port>]` 与 `[namespace]`，省略时用上面的默认值
（`pushrod_control` 另有一个在前的目标位置参数）。示例不带 `--help`，参数顺序以本手册
和源码为准：

```bash
./get_state 192.168.168.168:7447 robot168
python3 examples/get_state.py 192.168.168.168:7447 robot168
```

下文只写 C++ 形式（`./<name>`）；Python 载荷把命令换成
`python3 examples/<name>.py`，参数完全一样。

## 1. 确认连通：get_state

```bash
./get_state
```

它只查询状态、不下发任何目标。看到状态字段与电机列表，说明地址、namespace 与桥都对上了：

```
fsm_state      : ENABLED
arm_info       : ...
body_info      : ...
motors         : ...
  motor 1 pos=... mode=... confidence=...
feedback       : seq=... age=... ms
```

`body_info` 给出轮组、顶杆等身体部件的配置；每个电机的 `mode` 是机器人正在用的驱动
模式，`confidence` 是设备读回置信度——只有 `confirmed` 表示设备确认处在所报模式，
`requested` / `unknown` 下设备可能仍停在旧模式。在 ENABLED 阻尼态，`mode` 是按电机
家族自动识别的默认模式（灵足 mit、EYOU csp、其它 pp），所以刚跑完 `pp_control`
再查通常会看到 mit：那是回到阻尼态的结果，不是切换失败。`feedback` 的 `age` 是最近一帧关节
反馈的年龄，刚开始运行时可能还是 `none yet`，等一两百毫秒再运行一次。查询失败时示例
打印 SDK 返回的错误并以退出码 1 结束。

## 2. 观察遥测：get_joint

```bash
./get_joint
```

每 500 ms 打印一帧最新的关节快照，共 10 帧、5 秒。先打印一帧的 `fsm` / `seq` /
`age` 与电机数，再逐电机打印全部反馈字段：位置 `pos`（rad）、速度 `vel`（rad/s）、
力矩 `eff`（Nm）、温度 `temp`、使能 `enabled`、在线 `online` 与报警码 `fault`
（0 = 无故障）。
它只读反馈，不下发任何目标，适合在机器人旁确认链路与电机在线情况。

## 3. 认识状态机：set_fsm

机器人有五个状态：`STOP`（失能）、`ENABLED`（阻尼保持），以及三个模式态
`MIT` / `CSP` / `PP`。`set_fsm` 演示 STOP 与 ENABLED 之间的切换，**不会产生运动**：

```bash
./set_fsm
```

它先打印当前状态；当前不在 ENABLED 时先调用 `Enable()` 进入阻尼保持，等待 1 秒，
再调用 `Stop()` 回到失能，每次切换后打印新的 `fsm_state`。`ENABLED` 是阻尼保持：
电机通电、跟手有阻尼，但不会自己动。

## 4. 第一次运动：pp_control

`pp_control` 对每条臂的末端电机做一次来回：从当前位置走 `+0.3 rad`，再回到原位，
结束后切回 `ENABLED`。PP 是轮廓位置模式，规划在驱动侧完成，所以每程只发一帧
终点位置，不逐周期插值。灵足与 EYOU 两族臂都支持 PP。

**运行即动作**：先清空机械臂活动范围，并确认手臂已安装、有支撑；双臂机型两侧会同时动。

```bash
./pp_control
```

输出依次是 `arm_info` 与当前 `fsm_state`、解析出的臂型与自由度、每条臂每个电机的
`起始 -> 峰值 -> 起始`。扫动结束机械臂回到开始时的位置。示例不对位置做判定：是否真的
跟上了目标，用 `get_joint` 观察反馈；限位与安全策略在机器人端，被丢下的目标不会产生运动。

## 5. 其余三个运动示例

### mit_control

同一条来回路径，只是目标换成 MIT 形式（位置 + 速度前馈 + 刚度/阻尼），适合需要
柔顺行为的场合。MIT 只由灵足臂支持；EYOU 臂请用 `pp_control`。

### pushrod_control

把顶杆送到目标位置（mm，缺省 0 即零点）：

```bash
./pushrod_control 20        # 走到 20 mm
./pushrod_control           # 回到零点
```

使能模式下下发一帧 `SendBodyTarget`（只填顶杆字段），机器人端按 20 mm/s、50 mm/s²
的轮廓执行；随后 5 秒内每 500 ms 打印一次顶杆位置反馈。示例不对到位做判定。

### chassis_control

**运行即动作**：轮组正转 2 秒 → 停 1.5 秒 → 反转 2 秒，100 Hz 持续下发，结束后切回
`ENABLED`：

```bash
./chassis_control
```

轮组只在模式态生效，并且必须持续收到目标：机器人端 500 ms 收不到新消息就把轮速归零，
发一帧就等不会产生运动。示例在结束时补发 0.5 秒零速再切模式。

## 机器人端的几条规则

1. **目标只在对应模式态生效**：MIT 目标只在 FSM 状态为 `MIT` 时被接受，CSP 与 PP
   同理；模式态与驱动模式要同时匹配。
2. **目标不切换模式**：切模式只能经 `set_mode`（机器人端 `run_command` 服务）。
   切换是一段失能序列（停驱动 → 写电机模式 → 重新使能），机械臂短暂失去力矩；
   STOP 下先 `Enable()`，否则 `set_mode` 回 `busy`。`Enable(mode)` 不是切换：
   机器人不读 `enable` 的模式字段，落点仍是 `ENABLED`。
3. **ENABLED 是阻尼保持**：既不是「松手」，也不是「保持位置」；要保持位置就在 CSP
   或 PP 模式下持续下发目标。
4. **能力按臂型裁剪**：灵足臂支持 MIT / CSP / PP，EYOU 臂支持 CSP / PP；不支持的
   模式会被拒绝（`unsupported_mode`），示例打印错误并退出。
5. **轮组有 500 ms 死区**：机器人端 500 ms 收不到新的轮速目标就把轮速归零。
6. **顶杆与轮组的电机 id 按机型固定**：示例使用 17（顶杆）与 18、19（轮组）；
   换机型时先看 `get_state` 的 `body_info` 与电机列表。

## 自己写第一个程序

C++：把载荷目录加进你的工程，链接 `shidou::core`，然后：

```cpp
#include <cstdio>
#include <string>

#include "shidou/robot/robot.h"

// 地址与 namespace 是标识一台机器人的那一对；下面是出厂默认值。
const std::string address = "192.168.168.168:7447";
const std::string robot_namespace = "robot168";   // 与桥配置一致；无前缀时传 ""

shidou::robot::Robot robot(address, robot_namespace);  // 句柄自开会话，随作用域关闭
if (!robot.Ready()) {
    std::printf("robot init failed: %s\n", robot.LastError().c_str());
    return 1;
}

shidou::robot::RobotState state;
const shidou::robot::Result result = robot.GetRobotState(state);
if (!result) {
    std::printf("GetRobotState: %s\n", result.message.c_str());
    return 1;
}
std::printf("fsm_state=%s motors=%zu\n", state.fsm_state.c_str(),
            state.motor_ids.size());
```

Python：

```python
import shidou

robot = shidou.Robot(shidou.Config(robot_address="192.168.168.168:7447",
                                   namespace="robot168"))
if not robot.ready:
    raise SystemExit("connect failed: " + robot.last_error())

state = robot.get_state()
print(state.fsm_state, state.motor_ids)
```

构建接线（CMake / pip）与完整接口见载荷 README 的「在你的工程中使用」一节。

## 常见问题

- **启动即打印错误、没有任何状态字段**：地址或 namespace 与桥不一致，或桥没在跑。
  核对本手册「动手之前」的默认值，再用 `get_state` 单独排查。
- **连上了但等不到样本**：最常见的原因是 namespace 对不上；桥未启用 namespace 时
  要显式传空串 `""`。
- **`SetMode(MIT): unsupported_mode`（或其它 `SetMode(...)` 回执）**：该臂型
  不支持这个模式。灵足臂用 `mit_control`，两族臂都可以用 `pp_control`。
- **`no position for pushrod motor 17`**：顶杆不在 `joint_states` 里。确认机型带顶杆、
  机器人节点已启动，并先用 `get_state` 看 `body_info`。
- **电机不按预期运动**：先用 `get_joint` 观察位置反馈，确认 FSM 状态与模式态；注意
  `ENABLED` 本身不会产生运动。限位与安全策略在机器人端，目标可能被静默丢弃。
- **Windows 上运行示例**：在构建目录（`build/Release/` 或 Ninja 的 `build/`）里运行，
  `zenohc.dll` 构建时已复制到 exe 旁；手动搬运 exe 时要把 `bin/<平台>/zenohc.dll`
  一起带上。
