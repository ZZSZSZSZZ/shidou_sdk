// 关节状态监控示例：本示例演示如何使用SDK订阅遥测关节位置。
// 用 SetJointFeedbackCallback 订阅遥测，每 500 ms 用 LastFeedback 打印
// 实时快照，直到可选时长结束。
// [seconds]为程序遥测时长，当[seconds]为 0（或负数）表示一直运行到进程被杀 —— 适合在机器人旁边工作时挂一个长期监控。
//
// 当样本停止到达（如网络异常中断）时，缓存的最后一帧会带着不断增长的 age 和 STALE 标记继续打印，并触发一次 [WARN] 警告；
//
// 用法：get_joint [seconds] [<ip>:<port>] [namespace]
// namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
// namespace 时显式传空串：get_joint <seconds> <ip>:<port> ""
// 用法与缺省值也可以直接问示例：get_joint --help

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "example_common.h"
#include "shidou/comm/options.h"
#include "shidou/robot/robot.h"

namespace {

constexpr std::chrono::milliseconds kPrintInterval{500};
// 遥测是连续流；沉默超过该阈值即触发 stale 回调。
constexpr std::chrono::milliseconds kStaleThreshold{1000};

void PrintSnapshot(const shidou::msg::JointFeedback& fb, uint64_t seq,
                   double age_ms, const std::string& fsm, bool stale) {
    std::printf("fsm=%s seq=%llu age=%.1f ms%s motors=%zu\n", fsm.c_str(),
                static_cast<unsigned long long>(seq), age_ms,
                stale ? " [STALE]" : "", fb.motor_ids.size());
    for (size_t i = 0; i < fb.motor_ids.size(); ++i) {
        const double pos = i < fb.position.size() ? fb.position[i] : 0.0;
        const double vel = i < fb.velocity.size() ? fb.velocity[i] : 0.0;
        const double eff = i < fb.effort.size() ? fb.effort[i] : 0.0;
        const float tmp = i < fb.temperature.size() ? fb.temperature[i] : 0.0f;
        const uint32_t fault = i < fb.fault_code.size() ? fb.fault_code[i] : 0u;
        const uint8_t online = i < fb.online.size() ? fb.online[i] : 0u;
        std::printf("  motor %2u  pos=% 8.4f  vel=% 7.4f  eff=% 6.3f  temp=%5.1f  fault=0x%04x  online=%u\n",
                    fb.motor_ids[i], pos, vel, eff, tmp, fault, online);
    }
}

// 订阅与打印：每 500 ms 打一帧快照，直到 seconds 用完（<= 0 表示一直运行到进程被杀）。
void MonitorJoints(shidou::robot::Robot& robot, const shidou::comm::ZenohConfig& cfg,
                   example::Verdicts& verdicts, double seconds) {
    std::printf("monitoring %s for %.0f s (0 = until killed)...\n", cfg.robot_address.c_str(),
                seconds);

    // 精确接收计数：缓存只保留最新样本，轮询缓存会少算；回调每个样本触发一次。
    std::atomic<uint64_t> received{0};
    robot.SetJointFeedbackCallback(
        [&received](const shidou::msg::JointFeedback&) { ++received; });
    // 每次断流事件只警告一次（收到新样本后重新武装）。
    robot.SetFeedbackStaleCallback(kStaleThreshold, [&verdicts](double age_ms) {
        verdicts.Warn("no telemetry for %.0f ms (link or robot node down?)", age_ms);
    });

    bool have_feedback = false;
    double age_min = 0.0;
    double age_max = 0.0;
    const auto start = std::chrono::steady_clock::now();
    while (seconds <= 0.0 ||
           std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
               seconds) {
        std::this_thread::sleep_for(kPrintInterval);

        shidou::msg::JointFeedback fb;
        if (!robot.LastFeedback(fb)) {
            std::printf("no feedback yet\n");
            continue;
        }
        const double age = robot.FeedbackAgeMs();
        if (!have_feedback) {
            have_feedback = true;
            age_min = age_max = age;
        } else {
            if (age < age_min) age_min = age;
            if (age > age_max) age_max = age;
        }
        PrintSnapshot(fb, robot.FeedbackSeq(), age, robot.FsmState(),
                      age > static_cast<double>(kStaleThreshold.count()));
    }

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const uint64_t samples = received.load();
    if (samples == 0) {
        verdicts.Fail("no feedback received");
        return;
    }
    std::printf("received %llu samples in %.1f s (~%.1f Hz), age %.1f..%.1f ms\n",
                static_cast<unsigned long long>(samples), elapsed,
                elapsed > 0.0 ? samples / elapsed : 0.0, age_min, age_max);
    verdicts.Pass("get_joint completed");
}

} // namespace

int main(int argc, char** argv) {
    // 监控进程在 stdout 被重定向到文件时也必须即时打印（全缓冲会把输出
    // 拖到进程退出才落盘）。
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    // 运行时长（秒）；<= 0 表示一直运行到进程被杀。缺省值写在下面的参数声明里，
    // 与显式取值走同一条解析路径；非数字必须是非法调用——静默取 0 等于把有界监控
    // 变成一直运行到进程被杀。
    double seconds = 0.0;
    const example::PositionalArg seconds_arg{
        "seconds",
        "10",
        "telemetry duration in seconds; 0 or negative runs until the process is killed",
        example::ParseDouble("seconds", "a number", seconds)};

    return example::Run(argc, argv, {seconds_arg},
                        [&seconds](shidou::robot::Robot& robot,
                                   const shidou::comm::ZenohConfig& cfg,
                                   example::Verdicts& verdicts) {
        MonitorJoints(robot, cfg, verdicts, seconds);
    });
}
