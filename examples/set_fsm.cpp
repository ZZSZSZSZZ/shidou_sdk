// FSM 切换示例：本示例演示如何使用SDK在 STOP 与 ENABLED 状态间切换。
// 按回车在 STOP状态 与 ENABLED状态 之间来回切换，Ctrl+C 退出。
// 退出时 FSM 保持当前状态不变。
//
// 初始状态用 GetRobotState 查询，不假设订阅缓存里已有 fsm_state 样本。
//
// 用法：set_fsm [<ip>:<port>] [namespace]
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：set_fsm <ip>:<port> ""
// 用法与缺省值也可以直接问示例：set_fsm --help

#include <cstdio>
#include <string>

#include "example_common.h"
#include "shidou/comm/options.h"
#include "shidou/robot/robot.h"

namespace {

// 切换循环：回车切一次，stdin 关闭 / Ctrl+C / Ctrl+Z 退出（见 example_common.h）。
// 某次切换失败只记 [FAIL] 并继续接受下一次按键；退出码由判定积累给出，
// 操作者主动退出本身不算失败。
void ToggleFsm(shidou::robot::Robot& robot, const shidou::comm::ZenohConfig&,
               example::Verdicts& verdicts) {
    shidou::robot::RobotState state;
    const shidou::robot::Result state_result = robot.GetRobotState(state);
    if (!state_result) {
        verdicts.Fail("GetRobotState: %s", state_result.message.c_str());
        return;
    }
    // 初始不是 STOP（ENABLED 或任一控制模式）时，第一次按键先 Stop。
    bool stopped = (state.fsm_state == "STOP");
    std::printf("current fsm_state=%s\n", state.fsm_state.c_str());
    std::printf("press Enter to toggle STOP/ENABLED, Ctrl+C to exit\n");

    int toggles = 0;
    for (;;) {
        if (!example::WaitEnter()) {
            std::printf("toggled %d times, final fsm_state=%s\n", toggles,
                        robot.FsmState().c_str());
            return;
        }
        const shidou::robot::Result toggled = stopped ? robot.Enable() : robot.Stop();
        if (toggled) {
            stopped = (robot.FsmState() == "STOP");
            ++toggles;
            verdicts.Pass("%s -> fsm_state=%s", stopped ? "Enable" : "Stop",
                          robot.FsmState().c_str());
        } else {
            verdicts.Fail("%s: %s (state unchanged)", stopped ? "Enable" : "Stop",
                          toggled.message.c_str());
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    // FSM 切换没有自己的参数：位置参数只有 [<ip>:<port>] 与 [namespace]。
    return example::Run(argc, argv, {}, ToggleFsm);
}
