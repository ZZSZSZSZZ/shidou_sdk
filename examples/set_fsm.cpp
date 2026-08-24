// FSM 切换示例：本示例演示如何使用SDK在 STOP 与 ENABLED 状态间切换。
// 按回车在 STOP状态 与 ENABLED状态 之间来回切换，Ctrl+C 退出。
// 退出时 FSM 保持当前状态不变。
//
// 初始状态用 GetRobotState 查询，不假设订阅缓存里已有 fsm_state 样本。
//
// 用法：set_fsm [<ip>:<port>]

#include <cstdio>
#include <string>

#include "shidou/comm/zenoh_factory.h"
#include "shidou/robot/robot.h"

#if defined(_WIN32)
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace {

// 读取一次无缓冲按键；stdin 关闭时返回 EOF（脚本可以管道输入）。Ctrl+C
// 要么以 CTRL_C_EVENT 信号直接终止进程，要么以 0x03 按键码出现（由调用方按退出处理）。
int WaitKey() {
#if defined(_WIN32)
    return _getch();
#else
    struct termios old_t {};
    struct termios new_t {};
    tcgetattr(STDIN_FILENO, &old_t);
    new_t = old_t;
    new_t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_t);
    const int c = std::getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_t);
    return c;
#endif
}

} // namespace

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
        // Windows 下 _getch 的回车是 '\r'，POSIX 原始模式是 '\n'；其余按键一律忽略。
        int c = 0;
        while (c != '\r' && c != '\n') {
            c = WaitKey();
            // processed input 关闭时（ConPTY 宿主），Ctrl+C 不会产生 CTRL_C_EVENT 信号而是以 0x03 按键到达；
            // Ctrl+Z 是传统 EOF 键——两者与 stdin 关闭同等处理，直接退出。
            if (c == EOF || c == 0x03 || c == 0x1a) {
                std::printf("toggled %d times, final fsm_state=%s\n", toggles,
                            robot.FsmState().c_str());
                shidou::comm::ZenohFactory::Instance().Shutdown();
                return 0;
            }
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
