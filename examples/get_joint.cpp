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

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "shidou/comm/zenoh_factory.h"
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

} // namespace

int main(int argc, char** argv) {
    // 监控进程在 stdout 被重定向到文件时也必须即时打印（全缓冲会把输出
    // 拖到进程退出才落盘）。
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    // 运行时长（秒）；<= 0 表示一直运行到进程被杀。
    double seconds = 10.0;
    if (argc > 1) {
        seconds = std::atof(argv[1]);
    }
    std::string robot_address = "192.168.168.168:7447";
    if (argc > 2) {
        robot_address = argv[2];
    }
    std::string ns = "robot168";
    if (argc > 3) {
        ns = argv[3];
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
    std::printf("monitoring %s for %.0f s (0 = until killed)...\n", robot_address.c_str(), seconds);

    // 精确接收计数：缓存只保留最新样本，轮询缓存会少算；回调每个样本触发一次。
    std::atomic<uint64_t> received{0};
    robot.SetJointFeedbackCallback(
        [&received](const shidou::msg::JointFeedback&) { ++received; });
    // 每次断流事件只警告一次（收到新样本后重新武装）。
    robot.SetFeedbackStaleCallback(kStaleThreshold, [](double age_ms) {
        std::printf("[WARN] no telemetry for %.0f ms (link or robot node down?)\n", age_ms);
    });

    bool have_feedback = false;
    double age_min = 0.0;
    double age_max = 0.0;
    const auto start = std::chrono::steady_clock::now();
    while (seconds <= 0.0 ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <seconds) {
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
    shidou::comm::ZenohFactory::Instance().Shutdown();
    const uint64_t samples = received.load();
    if (samples == 0) {
        std::printf("[FAIL] no feedback received\n");
        return 1;
    }
    std::printf("received %llu samples in %.1f s (~%.1f Hz), age %.1f..%.1f ms\n",
                static_cast<unsigned long long>(samples), elapsed,
                elapsed > 0.0 ? samples / elapsed : 0.0, age_min, age_max);
    std::printf("[PASS] get_joint completed\n");
    return 0;
}
