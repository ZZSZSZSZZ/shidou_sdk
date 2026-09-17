// 顶杆示例：本示例演示如何使用SDK在使能模式下填写顶杆目标位置参数使顶杆前往目标点并读回反馈。
// 流程：GetRobotState 查询状态 → 不在 ENABLED 则先 Enable（全程保持在使能模式，不下发模式切换）→
// 从遥测反馈读顶杆当前位置 → 打印动作计划并等回车确认 → 填目标位置下发一帧 BodyTarget
// （只填顶杆字段；目标由命令行给出，缺省 0，即零点）→ 轮询反馈直到到位或超时 → 结束
// （单程，不回起点）。
//
// BodyTarget 的顶杆字段：pushrod_id 为顶杆电机号（0 = 本帧无顶杆目标），position 单位为
// mm，是绝对目标位置参数（填多少就走到哪）；velocity / acceleration 为设备侧轮廓速度与
// 加速度（mm/s、mm/s²）；轮组字段留空（空数组 = 本帧不动轮组）。顶杆在 ENABLED 与
// POSITION 下均生效、进入位置闭环后自保持，所以示例每次动作只填参数下发一帧，不逐周期重发。
// 顶杆反馈不在独立话题：它随 joint_states 一起到达，按 motor_id 与轮组区分（顶杆 mm）。
//
// 起点取反馈里的当前位置，所以**没有反馈样本时示例不动电机**并直接退出——不盲发目标。
// 目标位置的物理含义（哪一端是零点、位置增大朝哪走）由机器人端定义，示例只负责填参数。
//
// 用法：pushrod_control [target_mm] [<ip>:<port>] [namespace]
// target_mm 为目标位置参数（mm，绝对值，填多少就走到哪），缺省 0（零点）。
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：pushrod_control <target_mm> <ip>:<port> ""

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "example_common.h"
#include "shidou/comm/zenoh_factory.h"
#include "shidou/robot/robot.h"

namespace {

constexpr std::chrono::milliseconds kFeedbackPeriod{100};  // 到位判据的反馈轮询周期
constexpr uint32_t kPushrodId = 17;            // 顶杆电机号（示例值，按机器人实际配置修改）
constexpr double kDefaultTargetPositionMm = 0.0;  // 目标位置参数缺省值（命令行不填时取 0，即零点）
constexpr double kProfileVelocity = 20.0;      // 轮廓速度 mm/s（示例值）
constexpr double kProfileAcceleration = 50.0;  // 轮廓加速度 mm/s²（示例值）
constexpr double kPositionToleranceMm = 1.0;   // 到位判据：位置差不超过该值即算到位
constexpr double kMoveTimeoutMarginSeconds = 10.0;  // 到位等待超时余量（行程 / 轮廓速度 + 余量）
constexpr double kFeedbackWaitSeconds = 5.0;   // 等首帧反馈的上限（订阅建立到首帧到达的空窗）

// 从遥测反馈里按电机号取位置；没有反馈样本或样本中缺该电机时返回 false。
bool ReadPosition(const shidou::robot::Robot& robot, uint32_t motor_id, double& out) {
    shidou::msg::JointFeedback feedback;
    if (!robot.LastFeedback(feedback)) {
        return false;
    }
    for (size_t i = 0; i < feedback.motor_ids.size() && i < feedback.position.size(); ++i) {
        if (feedback.motor_ids[i] == motor_id) {
            out = feedback.position[i];
            return true;
        }
    }
    return false;
}

// 等首帧反馈：订阅建立到首帧 joint_states 到达之间有短暂空窗，立刻读会误判成“无反馈”。
// 在 kFeedbackWaitSeconds 内轮询，返回是否拿到该电机的位置；saw_sample 区分
// “一帧反馈都没有”（链路或节点没通）与“有反馈但没有该电机”（电机号配错）。
bool WaitFirstPosition(const shidou::robot::Robot& robot, uint32_t motor_id, double& out,
                       bool& saw_sample) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        if (ReadPosition(robot, motor_id, out)) {
            return true;
        }
        saw_sample = robot.FeedbackAgeMs() >= 0.0;  // 负值表示还没收到过任何样本
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= kFeedbackWaitSeconds) {
            return false;
        }
        std::this_thread::sleep_for(kFeedbackPeriod);
    }
}

// 下发一帧顶杆目标：只填顶杆字段，轮组字段留空（本帧不动轮组）。
bool SendPushrod(shidou::robot::Robot& robot, double position) {
    shidou::msg::BodyTarget target;
    target.pushrod_id = kPushrodId;
    target.position = position;
    target.velocity = kProfileVelocity;
    target.acceleration = kProfileAcceleration;
    return robot.SendBodyTarget(target);
}

// 轮询反馈直到位置进入容差或超时；返回是否到位，实测位置经 out 返回。
// 超时按本段行程（from → goal）与轮廓速度估算，避免长行程被固定超时误判为失败。
bool WaitInPosition(const shidou::robot::Robot& robot, double from, double goal, double& out) {
    const double timeout = std::fabs(goal - from) / kProfileVelocity + kMoveTimeoutMarginSeconds;
    const auto t0 = std::chrono::steady_clock::now();
    int polls = 0;
    for (;;) {
        double now = 0.0;
        if (ReadPosition(robot, kPushrodId, now)) {
            out = now;
            if (std::fabs(now - goal) <= kPositionToleranceMm) {
                return true;
            }
            if (++polls % 10 == 0) {  // 约每 1 s 打一行进度
                std::printf("  position=%.2f mm (goal %.2f mm)\n", now, goal);
            }
        }
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= timeout) {
            return false;
        }
        std::this_thread::sleep_for(kFeedbackPeriod);
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string robot_address = "192.168.168.168:7447";
    // 目标位置参数（mm，绝对值）；非法参数直接退出，避免把输入错误当成目标下发给电机。
    double target_mm = kDefaultTargetPositionMm;
    if (argc > 1) {
        char* end = nullptr;
        target_mm = std::strtod(argv[1], &end);
        if (end == argv[1] || *end != '\0') {
            std::printf("[FAIL] bad target_mm: %s (expected a number in mm)\n", argv[1]);
            return 1;
        }
    }
    if (argc > 2) {
        robot_address = argv[2];
    }
    std::string ns = "robot168";
    if (argc > 3) {
        ns = argv[3];
    }
    shidou::comm::ZenohConfig cfg;
    cfg.robot_address = robot_address;
    cfg.namespace_ = ns;
    shidou::InitLogging("info");

    shidou::robot::Robot robot(cfg);
    if (!robot.Ready()) {
        std::printf("[FAIL] %s\n", robot.LastError().c_str());
        return 1;
    }

    shidou::robot::RobotState state;
    if (!robot.GetRobotState(state)) {
        std::printf("[FAIL] GetRobotState: %s\n", robot.LastError().c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("fsm_state=%s\n", state.fsm_state.c_str());

    // 顶杆在 ENABLED 下即可接收目标位置参数，示例全程保持在使能模式，不下发模式切换；
    // 处于其他状态（POSITION / STOP）时先切回 ENABLED。
    if (state.fsm_state != shidou::robot::kFsmStateEnabled) {
        if (!robot.Enable()) {
            std::printf("[FAIL] Enable: %s\n", robot.LastError().c_str());
            shidou::comm::ZenohFactory::Instance().Shutdown();
            return 1;
        }
        std::printf("fsm_state=%s\n", robot.FsmState().c_str());
    }

    // 起点只能来自反馈：拿不到就什么都不发（盲发目标可能让顶杆从任意位置起跳）。
    double start = 0.0;
    bool saw_sample = false;
    if (!WaitFirstPosition(robot, kPushrodId, start, saw_sample)) {
        std::printf("[FAIL] no position for pushrod motor %u in %.0f s (%s): "
                    "nothing was commanded\n",
                    kPushrodId, kFeedbackWaitSeconds,
                    saw_sample ? "feedback has no such motor" : "no feedback sample");
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("plan: pushrod %u %.2f mm -> %.2f mm (profile %.0f mm/s, %.0f mm/s^2)\n",
                kPushrodId, start, target_mm, kProfileVelocity, kProfileAcceleration);
    std::printf("press Enter to start, Ctrl+C to exit\n");
    if (!example::WaitEnter()) {
        std::printf("aborted, nothing was commanded\n");
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 0;
    }

    double measured = 0.0;
    if (!SendPushrod(robot, target_mm)) {
        std::printf("[FAIL] SendBodyTarget: %s\n", robot.LastError().c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    if (!WaitInPosition(robot, start, target_mm, measured)) {
        std::printf("[FAIL] pushrod did not reach %.2f mm (measured %.2f mm)\n", target_mm, measured);
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("[PASS] pushrod %.2f mm -> %.2f mm (measured %.2f mm)\n", start, target_mm, measured);

    shidou::comm::ZenohFactory::Instance().Shutdown();
    return 0;
}
