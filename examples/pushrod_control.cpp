// 顶杆示例：在 ENABLED 下把顶杆送到目标位置（mm），随后每 500 ms 打印一次位置
// 反馈，共 5 秒。一帧 BodyTarget 只填顶杆字段，机器人端按示例给出的轮廓速度与
// 加速度执行；示例不做到位判定。
//
// 流程：解析目标 → 构造 Robot → 必要时使能 → 下发一帧目标 → 打印 5 秒反馈。
//
// 用法：pushrod_control [target_mm] [<ip>:<port>] [namespace]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "shidou/robot/robot.h"

namespace {

constexpr uint32_t kPushrodId = 17;            // 顶杆电机号（示例值，按机器人实际配置修改）
constexpr double kProfileVelocity = 20.0;      // 轮廓速度 mm/s（示例值）
constexpr double kProfileAcceleration = 50.0;  // 轮廓加速度 mm/s²（示例值）

// 从遥测反馈里按电机号取位置；没有样本或样本中缺该电机时返回 false。
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

// 下发一帧顶杆目标：position 单位 mm；轮组字段留空 = 本帧不动轮组。
shidou::robot::Result SendPushrod(shidou::robot::Robot& robot, double position) {
    shidou::msg::BodyTarget target;
    target.pushrod_id = kPushrodId;
    target.position = position;
    target.velocity = kProfileVelocity;
    target.acceleration = kProfileAcceleration;
    return robot.SendBodyTarget(target);
}

} // namespace

int main(int argc, char** argv) {
    // 缺省 0 = 零点；非法目标不能静默当 0 下发。
    const char* target_text = argc > 1 ? argv[1] : "0";
    char* end = nullptr;
    const double target_mm = std::strtod(target_text, &end);
    if (end == target_text || *end != '\0') {
        std::printf("bad target_mm: %s (expected a number in mm)\n", target_text);
        return 1;
    }
    const std::string address = argc > 2 ? argv[2] : "192.168.168.168:7447";
    const std::string robot_namespace = argc > 3 ? argv[3] : "robot168";

    shidou::robot::Robot robot(address, robot_namespace);
    if (!robot.Ready()) {
        std::printf("robot init failed: %s\n", robot.LastError().c_str());
        return 1;
    }

    shidou::robot::RobotState state;
    const shidou::robot::Result queried = robot.GetRobotState(state);
    if (!queried) {
        std::printf("GetRobotState: %s\n", queried.message.c_str());
        return 1;
    }
    std::printf("fsm_state=%s\n", state.fsm_state.c_str());

    // 顶杆在 ENABLED 下即可接收目标位置参数，示例不下发模式切换。
    if (state.fsm_state != shidou::robot::kFsmStateEnabled) {
        const shidou::robot::Result enabled = robot.Enable();
        if (!enabled) {
            std::printf("Enable: %s\n", enabled.message.c_str());
            return 1;
        }
        std::printf("fsm_state=%s\n", robot.FsmState().c_str());
    }

    const shidou::robot::Result sent = SendPushrod(robot, target_mm);
    if (!sent) {
        std::printf("SendBodyTarget: %s\n", sent.message.c_str());
        return 1;
    }
    std::printf("pushrod %u -> %.2f mm (profile %.0f mm/s, %.0f mm/s^2)\n", kPushrodId,
                target_mm, kProfileVelocity, kProfileAcceleration);

    // 每 500 ms 取一次反馈；顶杆位置随 joint_states 到达，按 motor_id 取。
    for (int frame = 0; frame < 10; ++frame) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        double position = 0.0;
        if (ReadPosition(robot, kPushrodId, position)) {
            std::printf("  pushrod position=%.2f mm\n", position);
        } else {
            std::printf("  no position for pushrod motor %u yet\n", kPushrodId);
        }
    }
    return 0;
}
