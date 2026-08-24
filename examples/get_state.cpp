// 状态查询示例：本示例演示如何使用SDK查询机器人状态。
// 用 GetRobotState 一次取回状态，逐字段打印。
//
// 用法：get_state [<ip>:<port>]

#include <cstdio>
#include <string>

#include "shidou/comm/zenoh_factory.h"
#include "shidou/robot/robot.h"

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

    // GetRobotState 返回服务字段与遥测缓存的合并结果；反馈字段来自遥测快照。
    shidou::robot::RobotState state;
    if (!robot.GetRobotState(state)) {
        std::printf("[FAIL] %s\n", robot.LastError().c_str());
        return 1;
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
                    static_cast<unsigned long long>(state.feedback_seq), state.feedback_age_ms);
    } else {
        std::printf("feedback       : none yet\n");
    }

    shidou::comm::ZenohFactory::Instance().Shutdown();
    std::printf("[PASS] get_state completed\n");
    return 0;
}
