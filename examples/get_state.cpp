// 状态查询示例：本示例演示如何使用SDK查询机器人状态。
// 用 GetRobotState 一次取回状态，逐字段打印。
//
// 用法：get_state [<ip>:<port>] [namespace]
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：get_state <ip>:<port> ""
// 用法与缺省值也可以直接问示例：get_state --help

#include <cstdio>
#include <string>

#include "example_common.h"
#include "shidou/comm/options.h"
#include "shidou/robot/robot.h"

namespace {

// 取一次状态并逐字段打印。
void QueryState(shidou::robot::Robot& robot, const shidou::comm::ZenohConfig&,
                example::Verdicts& verdicts) {
    // GetRobotState 返回服务字段与遥测缓存的合并结果；反馈字段来自遥测快照。
    // 结果值自带本次调用的失败原因，不必再读粘性错误记录。
    shidou::robot::RobotState state;
    const shidou::robot::Result state_result = robot.GetRobotState(state);
    if (!state_result) {
        verdicts.Fail("GetRobotState: %s", state_result.message.c_str());
        return;
    }

    std::printf("fsm_state      : %s\n", state.fsm_state.c_str());
    std::printf("arm_info       : %s\n", state.arm_info.c_str());
    std::printf("gripper_info   : %s\n", state.gripper_info.c_str());
    std::printf("library_status : 0x%04x\n", state.library_status);
    std::printf("control_freq_hz: %.1f\n", state.control_freq_hz);
    std::printf("cycle avg/max  : %.2f / %.2f ms\n", state.control_cycle_avg_ms,
                state.control_cycle_max_ms);
    std::printf("cycle overruns : %u\n", state.cycle_overruns);
    std::printf("motors         : %zu\n", state.motor_ids.size());
    for (size_t i = 0; i < state.motor_ids.size(); ++i) {
        const double pos = i < state.positions.size() ? state.positions[i] : 0.0;
        std::printf("  motor %u pos=%.4f\n", state.motor_ids[i], pos);
    }
    // 遥测快照：尚未收到样本时 has_feedback 为 false、age 为负。
    if (state.has_feedback) {
        std::printf("feedback       : seq=%llu age=%.1f ms\n",
                    static_cast<unsigned long long>(state.feedback_seq),
                    state.feedback_age_ms);
    } else {
        std::printf("feedback       : none yet\n");
    }

    verdicts.Pass("get_state completed");
}

} // namespace

int main(int argc, char** argv) {
    // 状态查询没有自己的参数：位置参数只有 [<ip>:<port>] 与 [namespace]。
    return example::Run(argc, argv, {}, QueryState);
}
