// 顶杆示例：本示例演示如何使用SDK把顶杆目标位置下发到机器人并读回反馈。
// 流程：GetRobotState 查询状态 → 不在 ENABLED 则先 Enable → SetMode(POSITION) →
// 从遥测反馈读顶杆当前位置 → 打印动作计划并等回车确认 → 每段下发一帧 BodyTarget
// （只填顶杆字段）→ 轮询反馈直到到位或超时 → 反向回原位 → 收尾切回 ENABLED。
//
// BodyTarget 的顶杆字段：pushrod_id 为顶杆电机号（0 = 本帧无顶杆目标），position
// 单位 mm，velocity / acceleration 为设备侧轮廓速度与加速度（mm/s、mm/s²）；轮组
// 字段留空（空数组 = 本帧不动轮组）。顶杆在 ENABLED 与 POSITION 下生效、进入位置
// 闭环后自保持，因此每段只下发一帧，不逐周期重发；本示例统一切到 POSITION（轮组只在
// POSITION 态生效，与底盘示例一致）。
// 顶杆反馈不在独立话题：它随 joint_states 一起到达，按 motor_id 与轮组区分（顶杆 mm）。
//
// 起点取反馈里的当前位置，所以**没有反馈样本时示例不动电机**并直接退出——不盲发目标。
// 动作朝位置增大方向走一段再回原位，方向语义由机器人端决定。
//
// 用法：pushrod_control [<ip>:<port>] [namespace]
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：pushrod_control <ip>:<port> ""

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include "example_common.h"
#include "shidou/comm/zenoh_factory.h"
#include "shidou/robot/robot.h"

namespace {

constexpr std::chrono::milliseconds kFeedbackPeriod{100};  // 到位判据的反馈轮询周期
constexpr uint32_t kPushrodId = 17;            // 顶杆电机号（示例值，按机器人实际配置修改）
constexpr double kStrokeMm = 100.0;             // 往返行程（示例值）
constexpr double kProfileVelocity = 20.0;      // 轮廓速度 mm/s（示例值）
constexpr double kProfileAcceleration = 50.0;  // 轮廓加速度 mm/s²（示例值）
constexpr double kPositionToleranceMm = 1.0;   // 到位判据：位置差不超过该值即算到位
constexpr double kMoveTimeoutSeconds = 15.0;   // 单段等待超时（行程 / 轮廓速度 + 余量）

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
bool WaitInPosition(const shidou::robot::Robot& robot, double goal, double& out) {
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
        if (elapsed >= kMoveTimeoutSeconds) {
            return false;
        }
        std::this_thread::sleep_for(kFeedbackPeriod);
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string robot_address = "192.168.168.168:7447";
    if (argc > 1) {
        robot_address = argv[1];
    }
    std::string ns = "robot168";
    if (argc > 2) {
        ns = argv[2];
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

    // 模式命令只能从 ENABLED 发起（STOP 态只接受 enabled，不能直接切模式）。
    if (state.fsm_state != shidou::robot::kFsmStateEnabled) {
        if (!robot.Enable()) {
            std::printf("[FAIL] Enable: %s\n", robot.LastError().c_str());
            shidou::comm::ZenohFactory::Instance().Shutdown();
            return 1;
        }
        std::printf("fsm_state=%s\n", robot.FsmState().c_str());
    }
    if (!robot.SetMode(shidou::robot::ControlMode::kPosition)) {
        std::printf("[FAIL] SetMode(POSITION): %s\n", robot.LastError().c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("fsm_state=%s\n", robot.FsmState().c_str());

    // 起点只能来自反馈：拿不到就什么都不发（盲发目标可能让顶杆从任意位置起跳）。
    double start = 0.0;
    if (!ReadPosition(robot, kPushrodId, start)) {
        std::printf("[FAIL] no feedback for pushrod motor %u: nothing was commanded\n",
                    kPushrodId);
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    const double extended = start + kStrokeMm;
    std::printf("plan: pushrod %u %.2f mm -> %.2f mm -> %.2f mm (profile %.0f mm/s, %.0f mm/s^2)\n",
                kPushrodId, start, extended, start, kProfileVelocity, kProfileAcceleration);
    std::printf("press Enter to start, Ctrl+C to exit\n");
    if (!example::WaitEnter()) {
        std::printf("aborted, nothing was commanded\n");
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 0;
    }

    double measured = 0.0;
    if (!SendPushrod(robot, extended)) {
        std::printf("[FAIL] SendBodyTarget: %s\n", robot.LastError().c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    if (!WaitInPosition(robot, extended, measured)) {
        std::printf("[FAIL] pushrod did not reach %.2f mm in %.0f s (measured %.2f mm)\n",
                    extended,
                    kMoveTimeoutSeconds, measured);
        robot.Enable();  // 已经动过，收尾切回 ENABLED
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("[PASS] pushrod %.2f mm -> %.2f mm (measured %.2f mm)\n", start, extended, measured);

    if (!SendPushrod(robot, start)) {
        std::printf("[FAIL] SendBodyTarget: %s\n", robot.LastError().c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    if (!WaitInPosition(robot, start, measured)) {
        std::printf("[FAIL] pushrod did not return to %.2f mm in %.0f s (measured %.2f mm)\n", start,
                    kMoveTimeoutSeconds, measured);
        robot.Enable();
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("[PASS] pushrod %.2f mm -> %.2f mm (measured %.2f mm)\n", extended, start, measured);

    // 收尾切回 ENABLED，机器人回到可再次接收模式命令的状态（顶杆保持当前位置）。
    const bool ok = robot.Enable();
    shidou::comm::ZenohFactory::Instance().Shutdown();
    if (!ok) {
        std::printf("[FAIL] Enable: %s\n", robot.LastError().c_str());
        return 1;
    }
    std::printf("[PASS] Enable -> fsm_state=%s\n", robot.FsmState().c_str());
    return 0;
}
