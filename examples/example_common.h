#pragma once

// 示例共用小工具：无缓冲按键读取与"等一次回车"。
//
// 需要人确认后才动作的示例（set_fsm / pushrod_control / chassis_control）
// 复用本文件，使"回车 = 继续、stdin 关闭或 Ctrl+C = 放弃"只有一处定义。
// 各示例与 CMakeLists.txt 同目录，直接 `#include "example_common.h"` 即可。

#include <cstdio>

#if defined(_WIN32)
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace example {

// 读取一次无缓冲按键；stdin 关闭时返回 EOF（脚本可以管道输入）。Ctrl+C
// 要么以 CTRL_C_EVENT 信号直接终止进程，要么以 0x03 按键码出现（由调用方按退出处理）。
inline int WaitKey() {
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

// 等一次回车：回车返回 true；stdin 关闭 / Ctrl+C / Ctrl+Z 返回 false（视为放弃）。
// Windows 下 _getch 的回车是 '\r'，POSIX 原始模式是 '\n'；其余按键一律忽略。
inline bool WaitEnter() {
    for (;;) {
        const int c = WaitKey();
        // processed input 关闭时（ConPTY 宿主），Ctrl+C 不会产生 CTRL_C_EVENT 信号而是以 0x03 按键到达；
        // Ctrl+Z 是传统 EOF 键——两者与 stdin 关闭同等处理。
        if (c == EOF || c == 0x03 || c == 0x1a) {
            return false;
        }
        if (c == '\r' || c == '\n') {
            return true;
        }
    }
}

} // namespace example
