// MIT 目标流示例：本示例演示如何使用SDK按臂布局对 MIT 关节做位置扫动。
// 流程：GetRobotState 查询状态 → 不在 ENABLED 则先 Enable → SetMode(POSITION) → 以 100 Hz 下发 SendJointMITTarget → 扫动结束切回 ENABLED。
// 扫动电机取每臂末端，电机号由 arm_info 的自由度得出（左臂 = dof，右臂 = dof + 7）；单臂扫动一个电机，双臂依次扫动（先左后右）。
// 每个电机从当前位置出发，直线插值走三段：当前位置 → 0.3 (1 秒) → 0.3 (1 秒) → 0 (1 秒)。JointMITTarget 字段：positions 为目标角，
// velocities 为速度前馈（取段内斜率），kps/kds 为关节刚度/阻尼（示例值），torques 留空即前馈力矩 0。
// 注意：模式命令只能从 ENABLED 发起（STOP 状态只接受 enabled），
// 切 POSITION 前先确认当前状态；扫动结束切回 ENABLED。
//
// 用法：mit_control [<ip>:<port>]

#include <array>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "shidou/comm/zenoh_factory.h"
#include "shidou/robot/robot.h"

namespace {

constexpr std::chrono::milliseconds kPeriod{10};  // 100 Hz 下发
constexpr double kSegmentSeconds = 1.0;           // 每段 1 秒
constexpr double kWaypointHigh = 0.3;             // 第一段终点 rad
constexpr double kWaypointLow = -0.3;             // 第二段终点 rad
constexpr double kWaypointHome = 0.0;             // 回零目标 rad
constexpr double kGainKp = 40.0;                  // 关节刚度（示例值，可按需调整）
constexpr double kGainKd = 2.0;                   // 关节阻尼（示例值，可按需调整）

// 扫动电机取每臂末端：末端电机号 = 自由度（左臂 dof、右臂 dof + 7）。
struct ArmLayout {
    bool has_left = false;
    bool has_right = false;
    int left_dof = 0;   // 左臂自由度（末端电机号 = left_dof）
    int right_dof = 0;  // 右臂自由度（末端电机号 = right_dof + 7）
};

// 解析 arm_info（"型号_布局_自由度"，布局段为 left / right / dual；
// 自由度段为 "5" / "7" 或双臂不同的 "5x7"（左x右））。
bool ParseArmLayout(const std::string& arm_info, ArmLayout& out) {
    std::vector<std::string> segs;
    std::string cur;
    for (char c : arm_info) {
        if (c == '_') {
            segs.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    segs.push_back(cur);

    // 自由度段解析："5" → 左=右=5；"5x7" → 左=5、右=7。非法段返回 false。
    const auto parse_dof = [](const std::string& s, int& left, int& right) {
        const size_t x = s.find('x');
        const std::string ls = (x == std::string::npos) ? s : s.substr(0, x);
        const std::string rs = (x == std::string::npos) ? s : s.substr(x + 1);
        const auto to_int = [](const std::string& t, int& v) {
            if (t.empty()) {
                return false;
            }
            int acc = 0;
            for (char c : t) {
                if (c < '0' || c > '9') {
                    return false;
                }
                acc = acc * 10 + (c - '0');
            }
            v = acc;
            return true;
        };
        return to_int(ls, left) && to_int(rs, right);
    };

    for (size_t i = 0; i + 1 < segs.size(); ++i) {
        if (segs[i] != "left" && segs[i] != "right" && segs[i] != "dual") {
            continue;  // 非布局段（如型号段）跳过，不解析其后的段
        }
        int l = 0;
        int r = 0;
        if (!parse_dof(segs[i + 1], l, r)) {
            return false;
        }
        if (segs[i] == "left") {
            out.has_left = true;
            out.left_dof = l;
            return true;
        }
        if (segs[i] == "right") {
            out.has_right = true;
            out.right_dof = r;
            return true;
        }
        if (segs[i] == "dual") {
            out.has_left = true;
            out.has_right = true;
            out.left_dof = l;
            out.right_dof = r;
            return true;
        }
    }
    return false;
}

// 从 get_state 快照中找电机当前位置；快照没有该电机时回退 0 并告警。
double CurrentPosition(const shidou::robot::RobotState& state, uint32_t motor_id) {
    for (size_t i = 0; i < state.motor_ids.size() && i < state.positions.size(); ++i) {
        if (state.motor_ids[i] == motor_id) {
            return state.positions[i];
        }
    }
    std::printf("[WARN] motor %u not in get_state snapshot, sweep from 0\n", motor_id);
    return 0.0;
}

// 单电机三段直线插值扫动：start → 0.3 (1 s) → -0.3 (1 s) → 0 (1 s)。
// 段内以实际经过时间为插值变量，睡眠抖动不累积；velocity 取段内斜率做
// 速度前馈，kp/kd 固定为示例值。每段结束补发终点帧，保证段间衔接与
// 最终停位精确。
bool SweepMotor(shidou::robot::Robot& robot, uint32_t motor_id, double start) {
    const std::array<double, 4> waypoints{start, kWaypointHigh, kWaypointLow, kWaypointHome};
    std::printf("motor %u: %.3f -> %.3f -> %.3f -> %.3f (%.0f s per segment)\n", motor_id,
                waypoints[0], waypoints[1], waypoints[2], waypoints[3], kSegmentSeconds);
    for (size_t seg = 0; seg + 1 < waypoints.size(); ++seg) {
        const double from = waypoints[seg];
        const double to = waypoints[seg + 1];
        const double slope = (to - from) / kSegmentSeconds;  // 段内速度前馈 rad/s
        const auto t0 = std::chrono::steady_clock::now();
        for (;;) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                                 .count();
            if (t >= kSegmentSeconds) {
                break;
            }
            shidou::msg::JointMITTarget target;
            target.motor_ids = {motor_id};
            target.positions = {from + (to - from) * (t / kSegmentSeconds)};
            target.velocities = {slope};
            target.kps = {kGainKp};
            target.kds = {kGainKd};
            // torques 不填 = 前馈力矩 0。
            if (!robot.SendJointMITTarget(target)) {
                std::printf("[FAIL] publish motor %u: %s\n", motor_id,
                            robot.LastError().c_str());
                return false;
            }
            std::this_thread::sleep_for(kPeriod);
        }
        shidou::msg::JointMITTarget end_frame;
        end_frame.motor_ids = {motor_id};
        end_frame.positions = {to};
        end_frame.velocities = {0.0};  // 段末速度前馈清零，停在终点
        end_frame.kps = {kGainKp};
        end_frame.kds = {kGainKd};
        if (!robot.SendJointMITTarget(end_frame)) {
            std::printf("[FAIL] publish motor %u: %s\n", motor_id, robot.LastError().c_str());
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::string robot_address = "192.168.168.168:7447";
    if (argc > 1) {
        robot_address = argv[1];
    }
    shidou::comm::ZenohConfig cfg;
    cfg.robot_address = robot_address;
    shidou::InitLogging("info");

    shidou::robot::Robot robot(cfg);
    if (!robot.Ready()) {
        std::printf("[FAIL] %s\n", robot.LastError().c_str());
        return 1;
    }
    // 先查状态：arm_info 决定扫动电机，fsm_state 决定是否需要先 Enable。
    shidou::robot::RobotState state;
    if (!robot.GetRobotState(state)) {
        std::printf("[FAIL] GetRobotState: %s\n", robot.LastError().c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("arm_info=%s\n", state.arm_info.c_str());
    std::printf("fsm_state=%s\n", state.fsm_state.c_str());

    // 模式命令只能从 ENABLED 发起（STOP 态只接受 enabled，不能直接切模式），
    // 当前不在 ENABLED 时先 Enable 握手。
    if (state.fsm_state != shidou::robot::kFsmStateEnabled) {
        if (!robot.Enable()) {
            std::printf("[FAIL] Enable: %s\n", robot.LastError().c_str());
            shidou::comm::ZenohFactory::Instance().Shutdown();
            return 1;
        }
        std::printf("fsm_state=%s\n", robot.FsmState().c_str());
    }

    ArmLayout layout;
    if (!ParseArmLayout(state.arm_info, layout)) {
        std::printf("[FAIL] arm_info=%s: cannot parse layout/dof\n", state.arm_info.c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    const char* layout_name =
        layout.has_left && layout.has_right ? "dual" : (layout.has_left ? "left" : "right");
    std::printf("arm layout: %s (left dof=%d, right dof=%d)\n", layout_name, layout.left_dof,
                layout.right_dof);

    // 末端电机号由自由度得出：左 = dof、右 = dof + 7；双臂依次（先左后右）。
    std::vector<uint32_t> motors;
    if (layout.has_left) {
        motors.push_back(static_cast<uint32_t>(layout.left_dof));
    }
    if (layout.has_right) {
        motors.push_back(static_cast<uint32_t>(layout.right_dof + 7));
    }

    // MIT 目标由 POSITION 状态消费：先握手切入 POSITION 模式，确认后开始扫动。
    if (!robot.SetMode(shidou::robot::ControlMode::kPosition)) {
        std::printf("[FAIL] SetMode(POSITION): %s\n", robot.LastError().c_str());
        shidou::comm::ZenohFactory::Instance().Shutdown();
        return 1;
    }
    std::printf("fsm_state=%s, sweeping %zu motor(s)\n", robot.FsmState().c_str(),
                motors.size());

    for (uint32_t motor_id : motors) {
        if (!SweepMotor(robot, motor_id, CurrentPosition(state, motor_id))) {
            shidou::comm::ZenohFactory::Instance().Shutdown();
            return 1;
        }
    }

    // 扫动结束切回 ENABLED，机器人回到可再次接收模式命令的状态。
    const bool ok = robot.Enable();
    shidou::comm::ZenohFactory::Instance().Shutdown();
    if (!ok) {
        std::printf("[FAIL] Enable: %s\n", robot.LastError().c_str());
        return 1;
    }
    std::printf("[PASS] Enable -> fsm_state=%s\n", robot.FsmState().c_str());
    return 0;
}
