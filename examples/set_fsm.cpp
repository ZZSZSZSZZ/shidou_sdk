// 状态机最小切换：先进入 ENABLED（保持态：电机通电、原地停住、不会自己动；
// 不带词元时保持律按臂型自动判定——灵足为速度阻尼、EYOU / 其它为位置保持），
// 再 Stop() 回失能。全程不下发运动目标。
//
// 流程：查询当前状态 → 不在 ENABLED 时 Enable → 等 1 秒 → Stop。
// 注意：Stop() 后电机失能，悬空的手臂会失去支撑。
//
// 用法：set_fsm [<ip>:<port>] [namespace]

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "shidou/robot/robot.h"

int main(int argc, char** argv) {
    const std::string address = argc > 1 ? argv[1] : "192.168.168.168:7447";
    const std::string robot_namespace = argc > 2 ? argv[2] : "robot168";

    shidou::robot::Robot robot(address, robot_namespace);
    if (!robot.Ready()) {
        std::printf("robot init failed: %s\n", robot.LastError().c_str());
        return 1;
    }

    // 先查状态，已经在 ENABLED 就不再重复使能。
    shidou::robot::RobotState state;
    const shidou::robot::Result queried = robot.GetRobotState(state);
    if (!queried) {
        std::printf("GetRobotState: %s\n", queried.message.c_str());
        return 1;
    }
    std::printf("fsm_state=%s\n", state.fsm_state.c_str());

    // 先进入 ENABLED，再 Stop() 回到失能，完成一次往返。
    if (state.fsm_state != shidou::robot::kFsmStateEnabled) {
        const shidou::robot::Result enabled = robot.Enable();
        if (!enabled) {
            std::printf("Enable: %s\n", enabled.message.c_str());
            return 1;
        }
        std::printf("fsm_state=%s\n", robot.FsmState().c_str());
        // 留 1 秒观察保持态，再退回失能。
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Stop 立即失能；示例到此结束。
    const shidou::robot::Result stopped = robot.Stop();
    if (!stopped) {
        std::printf("Stop: %s\n", stopped.message.c_str());
        return 1;
    }
    std::printf("fsm_state=%s\n", robot.FsmState().c_str());
    return 0;
}
