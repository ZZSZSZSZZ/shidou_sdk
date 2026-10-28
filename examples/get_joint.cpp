// 只读示例：每 500 ms 用 LastFeedback 打印一帧最新关节反馈，共 10 帧、5 秒。
// 逐电机打印位置、速度、力矩、使能、在线、故障码与温度，用来在机器人旁确认
// 遥测链路与电机在线情况。不下发任何目标。
//
// 流程：构造 Robot（订阅在后台线程收包）→ 循环取缓存快照 → 打印 seq/age 与
// 每个电机的全部反馈字段。
//
// 用法：get_joint [<ip>:<port>] [namespace]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "shidou/robot/robot.h"

namespace {

// 反馈的各数组按 motor_ids 索引对齐；个别数组缺项时给出缺省值，避免越界。
template <typename T>
T At(const std::vector<T>& values, size_t index, T fallback) {
    return index < values.size() ? values[index] : fallback;
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

    // 表头与数据行用同一组列宽，保证两处对齐。
    std::printf("%6s  %8s  %10s  %8s  %7s  %7s  %6s  %5s\n", "motor", "pos(rad)",
                "vel(rad/s)", "eff(Nm)", "temp(C)", "enabled", "online", "fault");

    // 10 帧 × 500 ms = 5 秒。
    for (int frame = 0; frame < 10; ++frame) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        shidou::msg::JointFeedback feedback;
        // LastFeedback 读的是缓存里的最新一帧；还没有样本时返回 false。
        if (!robot.LastFeedback(feedback)) {
            std::printf("no feedback yet\n");
            continue;
        }
        // age 是这帧的年龄；持续变大说明遥测断流。
        std::printf("fsm=%s seq=%llu age=%.1f ms motors=%zu\n", robot.FsmState().c_str(),
                    static_cast<unsigned long long>(robot.FeedbackSeq()),
                    robot.FeedbackAgeMs(), feedback.motor_ids.size());
        // enabled/online 是 0/1：enabled = 该电机已使能，online = 未禁用且状态新鲜；
        // fault 0 = 无故障。
        for (size_t i = 0; i < feedback.motor_ids.size(); ++i) {
            std::printf("%6u  %8.4f  %10.4f  %8.4f  %7.1f  %7u  %6u  %5u\n",
                        feedback.motor_ids[i],
                        At(feedback.position, i, 0.0),
                        At(feedback.velocity, i, 0.0),
                        At(feedback.effort, i, 0.0),
                        At(feedback.temperature, i, 0.0f),
                        static_cast<unsigned>(At(feedback.enabled, i, uint8_t{0})),
                        static_cast<unsigned>(At(feedback.online, i, uint8_t{0})),
                        At(feedback.fault_code, i, uint32_t{0}));
        }
    }
    return 0;
}
