// PP 关节示例：每条臂的末端电机从当前位置走一个来回（+0.3 rad 再回原位）。
// PP 的轮廓规划在驱动侧完成，所以每程只发一帧终点位置，不逐周期插值；
// 每帧带上速度上限与加速度，力矩上限取机器人端配置。灵足与 EYOU 两族臂都支持 PP。
//
// 流程：查询状态与臂型 → Enable(PP) → SetMode(PP) → 逐电机走一个来回 → Enable(PP)
// 回 ENABLED。
//
// 用法：pp_control [<ip>:<port>] [namespace]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "shidou/robot/robot.h"

namespace {

// 轮廓参数随每帧目标下发：velocity 是驱动侧规划用的速度上限，acceleration 是
// 轮廓加速度。示例放慢到 0.5 rad/s、1.0 rad/s²，来回看得清也方便在机器人旁观察；
// 不给这两个参数时取机器人端 position.velocity / position.acceleration，通常更快。
// 力矩上限留空（0 = 未提供），仍取机器人端 position.torque——它是安全限值，
// 示例不覆盖。
constexpr double kAmplitude = 0.3;             // 单程摆幅 rad（示例值）
constexpr double kProfileVelocity = 0.5;       // 速度上限 rad/s（示例值）
constexpr double kProfileAcceleration = 1.0;   // 轮廓加速度 rad/s²（示例值）

// 单程轮廓的理论时长：梯形速度曲线下 = 距离/速度 + 速度/加速度；距离短、到不了
// 速度上限的三角形曲线也以该值为上界。再乘余量，等驱动真正走完这一段。
constexpr double kSegmentSeconds =
    kAmplitude / kProfileVelocity + kProfileVelocity / kProfileAcceleration;
constexpr double kSegmentWaitMargin = 1.25;
constexpr std::chrono::duration<double> kSegmentWait{kSegmentSeconds * kSegmentWaitMargin};

// 解析十进制非负整数；arm_info 的自由度段用它读取。
bool ParseInt(const std::string& text, int& value) {
    if (text.empty()) {
        return false;
    }
    char* end = nullptr;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0' || parsed < 0) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

// arm_info 的解析结果：装了哪一侧、各侧自由度；末端电机号 = 左 dof、右 dof + 7。
struct ArmLayout {
    bool has_left = false;
    bool has_right = false;
    int left_dof = 0;
    int right_dof = 0;
};

// 从 arm_info 找出构型段：left_7 / right_7 / dual_5x7；旧版机器人带电机家族前缀
// （如 EYOU_left_7），非构型段跳过。解析失败返回 false。
bool ParseArmLayout(const std::string& arm_info, ArmLayout& layout) {
    std::stringstream stream(arm_info);
    std::vector<std::string> segments;
    for (std::string part; std::getline(stream, part, '_');) {
        segments.push_back(part);
    }
    for (size_t i = 0; i + 1 < segments.size(); ++i) {
        const std::string& side = segments[i];
        if (side != "left" && side != "right" && side != "dual") {
            continue;
        }
        const std::string& dof = segments[i + 1];
        const size_t x = dof.find('x');
        int left = 0;
        int right = 0;
        if (x == std::string::npos) {
            if (!ParseInt(dof, left)) {
                return false;
            }
            right = left;
        } else if (!ParseInt(dof.substr(0, x), left) ||
                   !ParseInt(dof.substr(x + 1), right)) {
            return false;
        }
        layout.has_left = side != "right";
        layout.has_right = side != "left";
        layout.left_dof = layout.has_left ? left : 0;
        layout.right_dof = layout.has_right ? right : 0;
        return true;
    }
    return false;
}

// 从 get_state 快照中找电机当前位置；快照没有该电机时回退 0。
double CurrentPosition(const shidou::robot::RobotState& state, uint32_t motor_id) {
    for (size_t i = 0; i < state.motor_ids.size() && i < state.positions.size(); ++i) {
        if (state.motor_ids[i] == motor_id) {
            return state.positions[i];
        }
    }
    std::printf("motor %u not in get_state snapshot, sweep from 0\n", motor_id);
    return 0.0;
}

// 单电机走一个来回：start → start + kAmplitude → start。每程只发一帧终点位置，
// 由驱动侧按帧里的速度上限与加速度规划整段轮廓；力矩上限取机器人端配置。
// 返回 false 表示下发失败，错误消息已打印。
bool SweepMotor(shidou::robot::Robot& robot, uint32_t motor_id, double start) {
    const double peak = start + kAmplitude;
    std::printf(
        "motor %u: %.3f -> %.3f -> %.3f (velocity %.2f rad/s, acceleration %.2f rad/s^2)\n",
        motor_id, start, peak, start, kProfileVelocity, kProfileAcceleration);
    for (const double position : {peak, start}) {
        // 位置是终点，速度与加速度决定驱动侧的轮廓；力矩上限留空 = 取机器人端配置。
        shidou::msg::JointPositionTarget target;
        target.motor_ids = {motor_id};
        target.positions = {position};
        target.velocities = {kProfileVelocity};
        target.accelerations = {kProfileAcceleration};
        const shidou::robot::Result sent = robot.SendJointPositionTarget(target);
        if (!sent) {
            std::printf("SendJointPositionTarget: %s\n", sent.message.c_str());
            return false;
        }
        // 等驱动把这一段走完，再发下一程。
        std::this_thread::sleep_for(kSegmentWait);
    }
    return true;
}

int SweepJoints(shidou::robot::Robot& robot) {
    shidou::robot::RobotState state;
    const shidou::robot::Result queried = robot.GetRobotState(state);
    if (!queried) {
        std::printf("GetRobotState: %s\n", queried.message.c_str());
        return 1;
    }
    std::printf("arm_info=%s\n", state.arm_info.c_str());
    std::printf("fsm_state=%s\n", state.fsm_state.c_str());

    // arm_info 决定哪几条臂参与扫动，以及各自的末端电机号。
    ArmLayout layout;
    if (!ParseArmLayout(state.arm_info, layout)) {
        std::printf("arm_info=%s: cannot parse layout/dof\n", state.arm_info.c_str());
        return 1;
    }
    std::printf("arm layout: %s (left dof=%d, right dof=%d)\n",
                layout.has_left && layout.has_right ? "dual"
                                                    : (layout.has_left ? "left" : "right"),
                layout.left_dof, layout.right_dof);

    std::vector<uint32_t> motors;
    if (layout.has_left) {
        motors.push_back(static_cast<uint32_t>(layout.left_dof));
    }
    if (layout.has_right) {
        motors.push_back(static_cast<uint32_t>(layout.right_dof + 7));
    }

    // 先使能，并把保持律选成 PP：落点仍是 ENABLED，只是停住时由驱动的位置闭环
    // 保持实测位置（不锁定目标，被推开就停在新位置）。从 STOP 出发必须走这一步：
    // STOP 下 SetMode 回 busy，因为模式态自己不会使能。
    const shidou::robot::Result enabled = robot.Enable(shidou::robot::ControlMode::kPp);
    if (!enabled) {
        std::printf("Enable(PP): %s\n", enabled.message.c_str());
        return 1;
    }
    // 再切 PP 状态：保持律已经是 PP，SetMode 只剩状态迁移，不再重复那段停驱动 →
    // 写电机模式 → 重新使能的序列。切换排队到当前运动停稳后执行。
    const shidou::robot::Result switched = robot.SetMode(shidou::robot::ControlMode::kPp);
    if (!switched) {
        std::printf("SetMode(PP): %s\n", switched.message.c_str());
        return 1;
    }
    // 打印状态，确认已在 PP。
    std::printf("fsm_state=%s\n", robot.FsmState().c_str());

    for (uint32_t motor_id : motors) {
        if (!SweepMotor(robot, motor_id, CurrentPosition(state, motor_id))) {
            return 1;
        }
    }

    // 回到 ENABLED：同样带上 PP 词元，停住时由驱动位置闭环保持实测位置，不会弹回
    // 扫动前的位置。最后一程刚结束时手臂可能还在动，机器人回 rejected_moving，
    // Enable 自动重发，停稳后落到 ENABLED。
    const shidou::robot::Result held = robot.Enable(shidou::robot::ControlMode::kPp);
    if (!held) {
        std::printf("Enable(PP): %s\n", held.message.c_str());
        return 1;
    }
    // 打印最终状态，确认回到 ENABLED。
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
    return SweepJoints(robot);
}
