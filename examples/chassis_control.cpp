// 底盘示例：本示例演示如何使用SDK驱动底盘轮组（BodyTarget 的轮组字段）并读回反馈。
// 流程：GetRobotState 查询状态 → 不在 ENABLED 则先 Enable → SetMode(POSITION) →
// 打印动作计划并等回车确认 → 以 100 Hz 持续下发轮速（正转 → 停 → 反转，每段先斜坡到
// 目标速度再保持）→ 每段结束按容差判定轮组反馈 → 停发一段验证死区 → 切回 ENABLED。
//
// BodyTarget 的轮组字段：wheel_ids 为轮毂电机号、velocities 为 rad/s，两者按索引对齐；
// max_currents 为电流上限（A，可短可空，缺省项用设备默认值）；顶杆字段留空
// （pushrod_id = 0 = 本帧无顶杆目标）。
//
// 轮组必须**持续下发**：机器人端按消息到达时刻计时，500 ms 无新消息就把轮速归零
// （死区），所以段内按固定周期连发（恒定值持续重发不会触发死区），而不是发一帧就等。
// 轮组只在 POSITION 态生效，其余状态下发会被强制归零。轮组反馈随 joint_states 到达，
// 按 motor_id 与顶杆区分（轮组 rad 与 rad/s）。收尾故意停发一段：超过死区窗口后轮速
// 应自行归零，不靠显式零速指令；只有仍未归零时才补发零速兜底。
//
// 用法：chassis_control [<ip>:<port>] [namespace]
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：chassis_control <ip>:<port> ""
// 用法与缺省值也可以直接问示例：chassis_control --help

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include "example_common.h"
#include "shidou/comm/options.h"
#include "shidou/robot/robot.h"

namespace {

constexpr std::chrono::milliseconds kPeriod{10};      // 100 Hz 下发
constexpr std::array<uint32_t, 2> kWheelIds{18, 19};  // 轮毂电机号
constexpr double kMaxCurrent = 5.0;                   // 每轮电流上限 A（示例值）
constexpr double kSpeed = 1.0;                        // 段内轮速幅值 rad/s（示例值）
constexpr double kRampSeconds = 1.0;                  // 每段从上一速度斜坡到目标速度的时长
constexpr double kStopSeconds = 1.0;                  // 停止段的保持时长
constexpr double kWheelTolerance = 0.3;               // 轮速判定容差 rad/s（示例值）
constexpr double kDeadmanSeconds = 1.5;               // 死区用例的停发时长（> 机器人端 500 ms 窗口）

// 动作段：先斜坡到 speed，再以 speed 保持 hold 秒。
struct Segment {
    const char* label;
    double speed;  // rad/s
    double hold;   // s
};

constexpr std::array<Segment, 3> kSegments{{
    {"forward", kSpeed, 1.5},
    {"stop", 0.0, kStopSeconds},
    {"reverse", -kSpeed, 1.5},
}};

// 下发一帧轮速：各轮同速、电流上限取示例值；顶杆字段留空 = 本帧不动顶杆。
// 返回本次下发的结果值，失败时 message 就是这一次的原因。
shidou::robot::Result SendWheels(shidou::robot::Robot& robot, double speed) {
    shidou::msg::BodyTarget target;
    target.wheel_ids.assign(kWheelIds.begin(), kWheelIds.end());
    target.velocities.assign(kWheelIds.size(), speed);
    target.max_currents.assign(kWheelIds.size(), kMaxCurrent);
    return robot.SendBodyTarget(target);
}

// 以固定周期持续下发同一速度，持续 seconds 秒；返回本次循环的结果值（成功时为空值）。
shidou::robot::Result PublishLoop(shidou::robot::Robot& robot, double speed, double seconds) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        const double t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (t >= seconds) {
            return {};
        }
        const shidou::robot::Result sent = SendWheels(robot, speed);
        if (!sent) {
            return sent;
        }
        std::this_thread::sleep_for(kPeriod);
    }
}

// 打印反馈里的各轮（位置 rad、速度 rad/s），并与目标速度按 kWheelTolerance 比较；
// 返回超出容差的轮数。反馈样本里没有轮组时（假节点只发关节电机）只报未判定，不算失败。
int PrintWheelFeedback(const shidou::robot::Robot& robot, double expected_vel) {
    shidou::msg::JointFeedback feedback;
    if (!robot.LastFeedback(feedback)) {
        std::printf("  wheels not judged: no feedback sample\n");
        return 0;
    }
    int judged = 0;
    int off = 0;
    for (uint32_t id : kWheelIds) {
        for (size_t i = 0; i < feedback.motor_ids.size(); ++i) {
            if (feedback.motor_ids[i] != id) {
                continue;
            }
            const double pos = i < feedback.position.size() ? feedback.position[i] : 0.0;
            const double vel = i < feedback.velocity.size() ? feedback.velocity[i] : 0.0;
            const bool ok = std::fabs(vel - expected_vel) <= kWheelTolerance;
            ++judged;
            if (!ok) {
                ++off;
            }
            std::printf("  wheel %u: pos=%.3f rad vel=%.3f rad/s%s\n", id, pos, vel,
                        ok ? "" : " [off target]");
            break;
        }
    }
    if (judged == 0) {
        std::printf("  wheels not judged: none of %zu in feedback\n", kWheelIds.size());
    } else if (off == 0) {
        std::printf("  wheels ok: %d/%zu within %.3f rad/s of %.3f\n", judged, kWheelIds.size(),
                    kWheelTolerance, expected_vel);
    } else {
        std::printf("  wheels off target: %d of %d judged\n", off, judged);
    }
    return off;
}

// 动作序列：确认计划 → 三段轮速（斜坡 + 保持）→ 死区用例 → 切回 ENABLED。
void DriveChassis(shidou::robot::Robot& robot, const shidou::comm::ZenohConfig&,
                  example::Verdicts& verdicts) {
    shidou::robot::RobotState state;
    const shidou::robot::Result state_result = robot.GetRobotState(state);
    if (!state_result) {
        verdicts.Fail("GetRobotState: %s", state_result.message.c_str());
        return;
    }
    std::printf("fsm_state=%s\n", state.fsm_state.c_str());

    // 模式命令只能从 ENABLED 发起（STOP 态只接受 enabled，不能直接切模式）。
    if (state.fsm_state != shidou::robot::kFsmStateEnabled) {
        const shidou::robot::Result enabled = robot.Enable();
        if (!enabled) {
            verdicts.Fail("Enable: %s", enabled.message.c_str());
            return;
        }
        std::printf("fsm_state=%s\n", robot.FsmState().c_str());
    }
    // 轮组只在 POSITION 态生效：切模式并确认后再发轮速。
    const shidou::robot::Result mode = robot.SetMode(shidou::robot::ControlMode::kPosition);
    if (!mode) {
        verdicts.Fail("SetMode(POSITION): %s", mode.message.c_str());
        return;
    }
    std::printf("fsm_state=%s\n", robot.FsmState().c_str());

    double planned = 0.0;
    for (const Segment& seg : kSegments) {
        planned += kRampSeconds + seg.hold;
    }
    std::printf("plan: wheels %u/%u, %zu segments (%.0f s): +%.1f -> 0 -> %.1f rad/s, then %.1f s "
                "without publishing (deadman)\n",
                kWheelIds[0], kWheelIds[1], kSegments.size(), planned, kSpeed, -kSpeed,
                kDeadmanSeconds);
    std::printf("the robot WILL move, keep the area clear\n");
    std::printf("press Enter to start, Ctrl+C to exit\n");
    if (!example::WaitEnter()) {
        std::printf("aborted, nothing was commanded\n");
        return;
    }

    bool wheels_ok = true;  // 任一段或死区用例的轮速判定失败即置 false
    double previous = 0.0;
    for (const Segment& seg : kSegments) {
        std::printf("segment %s: %.2f -> %.2f rad/s (ramp %.0f s, hold %.1f s)\n", seg.label,
                    previous, seg.speed, kRampSeconds, seg.hold);
        // 斜坡段按实际经过时间插值，睡眠抖动不累积。
        const auto t0 = std::chrono::steady_clock::now();
        for (;;) {
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (t >= kRampSeconds) {
                break;
            }
            const shidou::robot::Result sent =
                SendWheels(robot, previous + (seg.speed - previous) * (t / kRampSeconds));
            if (!sent) {
                verdicts.Fail("SendBodyTarget: %s", sent.message.c_str());
                return;
            }
            std::this_thread::sleep_for(kPeriod);
        }
        const shidou::robot::Result held = PublishLoop(robot, seg.speed, seg.hold);
        if (!held) {
            verdicts.Fail("SendBodyTarget: %s", held.message.c_str());
            return;
        }
        if (PrintWheelFeedback(robot, seg.speed) > 0) {
            wheels_ok = false;
        }
        previous = seg.speed;
    }

    // 死区用例：停发 kDeadmanSeconds（超过机器人端 500 ms 窗口），期间一帧不发，轮速应
    // 自行归零；只有仍偏离 0 时才补发零速兜底（否则连发零速会掩盖死区是否真的生效）。
    std::printf("deadman: %.1f s without publishing, wheels should return to 0 rad/s\n",
                kDeadmanSeconds);
    std::this_thread::sleep_for(std::chrono::duration<double>(kDeadmanSeconds));
    const int deadman_off = PrintWheelFeedback(robot, 0.0);
    if (deadman_off > 0) {
        wheels_ok = false;
        const shidou::robot::Result zeroed = PublishLoop(robot, 0.0, kStopSeconds);
        if (!zeroed) {
            verdicts.Fail("SendBodyTarget: %s", zeroed.message.c_str());
            return;
        }
    }

    // 切回 ENABLED，机器人回到可再次接收模式命令的状态。
    const shidou::robot::Result enabled = robot.Enable();
    if (!enabled) {
        verdicts.Fail("Enable: %s", enabled.message.c_str());
        return;
    }
    verdicts.Pass("Enable -> fsm_state=%s", robot.FsmState().c_str());
    if (!wheels_ok) {
        verdicts.Fail("wheel velocities off target (see above)");
        return;
    }
}

} // namespace

int main(int argc, char** argv) {
    // 底盘控制没有自己的参数：位置参数只有 [<ip>:<port>] 与 [namespace]。
    return example::Run(argc, argv, {}, DriveChassis);
}
