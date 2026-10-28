// 底盘示例：轮组正转 2 秒、停 1.5 秒、反转 2 秒，100 Hz 持续下发，最后切回 ENABLED。
// 轮组只在模式态生效，且必须持续收到目标：机器人端 500 ms 收不到新帧就把轮速归零。
//
// 流程：使能 → 切 PP → 逐段持续下发 → 补零速 → 切回 ENABLED。
// 注意：示例会驱动底盘，运行前留出空间并确认急停可用。
//
// 用法：chassis_control [<ip>:<port>] [namespace]

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include "shidou/robot/robot.h"

namespace {

constexpr std::chrono::milliseconds kPeriod{10};      // 目标下发周期：10 ms = 100 Hz
constexpr std::array<uint32_t, 2> kWheelIds{18, 19};  // 轮毂电机号（示例值，按实机核对）
constexpr double kMaxCurrent = 5.0;                   // 每轮电流上限 A（示例值）
constexpr double kSpeed = 0.5;                        // 轮速幅值 rad/s（示例值）

// 一段动作：按 speed 持续下发 hold 秒，标签只用于打印。
struct Segment {
    const char* label;
    double speed;  // rad/s
    double hold;   // s
};

constexpr std::array<Segment, 3> kSegments{{
    {"forward", kSpeed, 2.0},
    {"stop", 0.0, 1.5},
    {"reverse", -kSpeed, 2.0},
}};

// 下发一帧轮速（rad/s）：各轮同速、电流上限取示例值；顶杆字段留空 = 本帧不动顶杆。
shidou::robot::Result SendWheels(shidou::robot::Robot& robot, double speed) {
    shidou::msg::BodyTarget target;
    target.wheel_ids.assign(kWheelIds.begin(), kWheelIds.end());
    target.velocities.assign(kWheelIds.size(), speed);
    target.max_currents.assign(kWheelIds.size(), kMaxCurrent);
    return robot.SendBodyTarget(target);
}

// 以固定周期持续下发同一速度，持续 seconds 秒；任一次下发失败立即返回错误。
shidou::robot::Result PublishLoop(shidou::robot::Robot& robot, double speed, double seconds) {
    const auto start = std::chrono::steady_clock::now();
    for (;;) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= seconds) {
            return {};
        }
        const shidou::robot::Result sent = SendWheels(robot, speed);
        if (!sent) {
            return sent;
        }
        std::this_thread::sleep_for(kPeriod);
    }
}

int DriveChassis(shidou::robot::Robot& robot) {
    // 轮组只在模式态生效：先使能，再切 PP（STOP 下 set_mode 回 busy）。
    const shidou::robot::Result enabled = robot.Enable();
    if (!enabled) {
        std::printf("Enable: %s\n", enabled.message.c_str());
        return 1;
    }
    const shidou::robot::Result mode = robot.SetMode(shidou::robot::ControlMode::kPp);
    if (!mode) {
        std::printf("SetMode(PP): %s\n", mode.message.c_str());
        return 1;
    }
    std::printf("fsm_state=%s\n", robot.FsmState().c_str());

    for (const Segment& segment : kSegments) {
        std::printf("segment %s: %.2f rad/s for %.1f s\n", segment.label, segment.speed,
                    segment.hold);
        const shidou::robot::Result sent = PublishLoop(robot, segment.speed, segment.hold);
        if (!sent) {
            std::printf("SendBodyTarget: %s\n", sent.message.c_str());
            return 1;
        }
    }

    // 补发 0.5 秒零速，避免轮组带着旧目标进入 ENABLED。
    const shidou::robot::Result zeroed = PublishLoop(robot, 0.0, 0.5);
    if (!zeroed) {
        std::printf("SendBodyTarget: %s\n", zeroed.message.c_str());
        return 1;
    }

    // 回到 ENABLED。
    const shidou::robot::Result held = robot.Enable();
    if (!held) {
        std::printf("Enable: %s\n", held.message.c_str());
        return 1;
    }
    std::printf("fsm_state=%s\n", robot.FsmState().c_str());
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const std::string address = argc > 1 ? argv[1] : "192.168.168.168:7447";
    const std::string robot_namespace = argc > 2 ? argv[2] : "robot168";

    shidou::robot::Robot robot(address, robot_namespace);
    if (!robot.Ready()) {
        std::printf("robot init failed: %s\n", robot.LastError().c_str());
        return 1;
    }
    return DriveChassis(robot);
}
