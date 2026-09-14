// FSM 切换示例：本示例演示如何使用SDK在 STOP 与 ENABLED 状态间切换。
// 按回车在 STOP状态 与 ENABLED状态 之间来回切换，Ctrl+C 退出。
// 退出时 FSM 保持当前状态不变。
//
// 初始状态用 GetRobotState 查询，不假设订阅缓存里已有 fsm_state 样本。
//
// 用法：set_fsm [<ip>:<port>] [namespace]
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：set_fsm <ip>:<port> ""

#include <cstdio>
#include <string>

#include "example_common.h"
#include "shidou/comm/zenoh_factory.h"
#include "shidou/robot/robot.h"

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
        return 1;
    }
    // 初始不是 STOP（ENABLED 或任一控制模式）时，第一次按键先 Stop。
    bool stopped = (state.fsm_state == "STOP");
    std::printf("current fsm_state=%s\n", state.fsm_state.c_str());
    std::printf("press Enter to toggle STOP/ENABLED, Ctrl+C to exit\n");

    int toggles = 0;
    for (;;) {
        // 回车 = 切换一次；stdin 关闭 / Ctrl+C / Ctrl+Z = 退出（见 example_common.h）。
        if (!example::WaitEnter()) {
            std::printf("toggled %d times, final fsm_state=%s\n", toggles,
                        robot.FsmState().c_str());
            shidou::comm::ZenohFactory::Instance().Shutdown();
            return 0;
        }
        const bool ok = stopped ? robot.Enable() : robot.Stop();
        if (ok) {
            stopped = (robot.FsmState() == "STOP");
            ++toggles;
            std::printf("[PASS] %s -> fsm_state=%s\n",
                        stopped ? "Enable" : "Stop", robot.FsmState().c_str());
        } else {
            std::printf("[FAIL] %s: %s (state unchanged)\n",
                        stopped ? "Enable" : "Stop", robot.LastError().c_str());
        }
    }
}
