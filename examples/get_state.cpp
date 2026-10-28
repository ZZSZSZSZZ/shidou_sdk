// 只读示例：查询一次机器人状态并逐字段打印，用于确认地址、namespace 与机器人端
// 桥是否对得上（FSM 状态、臂型/机身构型、电机列表、最近一帧遥测的年龄）。
// 不下发任何目标。
//
// 流程：构造 Robot（自开会话）→ GetRobotState 服务查询 → 逐字段打印。
//
// 用法：get_state [<ip>:<port>] [namespace]

#include <cstdio>
#include <string>

#include "shidou/robot/robot.h"

int main(int argc, char** argv) {
    // 可选参数：机器人地址与 namespace，省略时用出厂默认值。
    const std::string address = argc > 1 ? argv[1] : "192.168.168.168:7447";
    // namespace 必须与机器人侧桥配置完全一致；桥未启用 namespace 时传空串。
    const std::string robot_namespace = argc > 2 ? argv[2] : "robot168";

    // Robot 自开会话，会话随 Robot 在作用域退出时关闭。
    shidou::robot::Robot robot(address, robot_namespace);
    if (!robot.Ready()) {
        std::printf("robot init failed: %s\n", robot.LastError().c_str());
        return 1;
    }

    // 服务查询：一次取回 FSM 状态、配置字段与最新遥测快照；超时或被拒答时
    // 返回错误，消息里带机器人端的回执。
    shidou::robot::RobotState state;
    const shidou::robot::Result result = robot.GetRobotState(state);
    if (!result) {
        std::printf("GetRobotState: %s\n", result.message.c_str());
        return 1;
    }

    std::printf("fsm_state      : %s\n", state.fsm_state.c_str());
    std::printf("arm_info       : %s\n", state.arm_info.c_str());
    std::printf("gripper_info   : %s\n", state.gripper_info.c_str());
    // 型号名：手写配置通道下为空串。
    std::printf("model          : %s\n", state.model.c_str());
    // 机身构型：none / wheels / pushrod / wheels_pushrod 的组合。
    std::printf("body_info      : %s\n", state.body_info.c_str());
    std::printf("library_status : 0x%04x\n", state.library_status);
    std::printf("control_freq_hz: %.1f\n", state.control_freq_hz);
    std::printf("cycle avg/max  : %.2f / %.2f ms\n", state.control_cycle_avg_ms,
                state.control_cycle_max_ms);
    std::printf("cycle overruns : %u\n", state.cycle_overruns);
    // 电机列表与位置来自服务快照；顺序不保证与遥测一致，跨来源按 motor_ids 对齐。
    // mode/confidence 也按 motor_ids 对齐，机器人不上报时显示 "-"；只有
    // confirmed 表示设备确认处在所报模式，其余情况下设备可能仍停在旧模式。
    std::printf("motors         : %zu\n", state.motor_ids.size());
    for (size_t i = 0; i < state.motor_ids.size(); ++i) {
        const double position = i < state.positions.size() ? state.positions[i] : 0.0;
        const std::string mode =
            i < state.control_mode.size() ? state.control_mode[i] : std::string("-");
        const std::string confidence = i < state.control_mode_confidence.size()
                                           ? state.control_mode_confidence[i]
                                           : std::string("-");
        std::printf("  motor %u pos=%.4f mode=%s confidence=%s\n", state.motor_ids[i],
                    position, mode.c_str(), confidence.c_str());
    }
    // 遥测快照：尚未收到样本时 has_feedback 为 false、age 为负。
    if (state.has_feedback) {
        std::printf("feedback       : seq=%llu age=%.1f ms\n",
                    static_cast<unsigned long long>(state.feedback_seq),
                    state.feedback_age_ms);
    } else {
        std::printf("feedback       : none yet\n");
    }
    return 0;
}
