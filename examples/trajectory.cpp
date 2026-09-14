// 轨迹控制示例：本示例演示如何使用SDK上传并执行关节轨迹。
// 通过 joint_trajectory 服务上传轨迹，切换到 TRAJECTORY 模式，观察关节
// 反馈是否收敛到目标位，然后 Stop。
//
// 收敛观察结束后由示例主动 Stop 退出。收敛判定仅供参考：电机不在位时
// 反馈永远不动，预算超时后直接 Stop。
//
// 用法：trajectory [<ip>:<port>] [namespace]
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：trajectory <ip>:<port> ""

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "shidou/comm/zenoh_factory.h"
#include "shidou/robot/robot.h"

namespace {

constexpr double kTarget = 0.1;    // 与下方上传的路径点一致
constexpr double kTolerance = 0.02;  // rad，目标附近的收敛窗口
constexpr std::chrono::seconds kSettleBudget{15};

shidou::msg::JointTrajectory MakeTrajectory() {
    shidou::msg::JointTrajectory traj;
    traj.header.frame_id = "example";
    traj.joint_id = {1, 2};
    // 轨迹包含单个 CSP 路径点（关节 1、2），MIT 路径点留空。
    shidou::msg::CSPWaypoint csp;
    csp.positions = {kTarget, kTarget};
    csp.max_velocity = {1.0, 1.0};
    csp.max_acceleration = {0.5, 0.5};
    traj.csp_points = {csp};
    return traj;
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
    if (!robot.Enable()) {
        std::printf("[FAIL] Enable: %s\n", robot.LastError().c_str());
        return 1;
    }

    if (!robot.UploadTrajectory(MakeTrajectory())) {
        std::printf("[FAIL] %s\n", robot.LastError().c_str());
        return 1;
    }
    std::printf("[PASS] trajectory accepted by the robot\n");

    if (!robot.SetMode(shidou::robot::ControlMode::kTrajectory)) {
        std::printf("[FAIL] %s\n", robot.LastError().c_str());
        return 1;
    }
    std::printf("[PASS] fsm_state=%s, watching for settle...\n",
                robot.FsmState().c_str());

    // 通过关节反馈观察完成：要么目标收敛，要么预算超时。
    const auto deadline = std::chrono::steady_clock::now() + kSettleBudget;
    bool settled = false;
    while (std::chrono::steady_clock::now() < deadline) {
        shidou::msg::JointFeedback fb;
        if (robot.LastFeedback(fb)) {
            bool on_target = fb.motor_ids.size() >= 2;
            for (size_t i = 0; i < fb.motor_ids.size(); ++i) {
                if ((fb.motor_ids[i] == 1 || fb.motor_ids[i] == 2) &&
                    std::abs(fb.position[i] - kTarget) > kTolerance) {
                    on_target = false;
                }
            }
            if (on_target) {
                settled = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (settled) {
        std::printf("[INFO] joints 1,2 settled within %.2f rad of targets\n", kTolerance);
    } else {
        std::printf("[INFO] not settled within %lld s (motors offline, or "
                    "trajectory params prevent arrival)\n",
                    static_cast<long long>(kSettleBudget.count()));
    }

    const bool ok = robot.Stop();
    shidou::comm::ZenohFactory::Instance().Shutdown();
    if (!ok) {
        std::printf("[FAIL] Stop: %s\n", robot.LastError().c_str());
        return 1;
    }
    std::printf("[PASS] Stop -> fsm_state=%s\n", robot.FsmState().c_str());
    return 0;
}
