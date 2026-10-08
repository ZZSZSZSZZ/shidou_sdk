#pragma once

// 示例共用 harness：operator 契约在本语言的实现。
//
// 示例对 operator 承诺的面在这里集中定义：`-h/--help` 的用法文本（示例自身的参数在
// 前，尾部两个参数在后，含缺省值）、地址与 namespace 的解析、生命周期骨架（日志初始化
// → 会话与 Robot 构造 → Ready() 失败时一行 [FAIL] 并退出 1）、判定前缀词汇与退出码积
// 累，以及等待一次回车。
//
// 这些条款的取值不在这里手写：下方生成区是两语言共用的 operator 契约条款真源的渲染
// 结果（真源不随示例发布），手改会被视为"派生副本过期"。退出码口径与放弃语义是行为
// 而非取值，两语言各自实现、按同一份真源核对：0 = 全部判定通过（含操作者放弃）；
// 1 = 有判定失败或非法调用。
//
// 每个示例只声明一次自己的参数（见 PositionalArg）：解析与 --help 用法文本都由同
// 一份声明派生，不另写用法字符串。main 里只剩业务动作：Run() 就绪后示例只负责下发与
// 判定，判定用 Verdicts 输出，退出码由判定积累，示例不自己拼退出码。
//
// 会话归 harness 所有：Run() 打开它、交给 Robot，body 的每条退出路径（含早退）都回到
// 这里，作用域末尾随局部对象自然释放，示例不自己关会话。
//
// 需要人确认后才动作的示例（set_fsm / pushrod_control / chassis_control）复用本文件
// 的按键读取，使"回车 = 继续、stdin 关闭或 Ctrl+C = 放弃"只有一处定义。各示例与
// 本文件同目录，直接 `#include "example_common.h"` 即可。

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "shidou/comm/zenoh_session.h"
#include "shidou/logging.h"
#include "shidou/robot/robot.h"

#if defined(_WIN32)
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

// 让编译器核对判定行的格式串（与 std::printf 同一套）。MSVC 没有对应属性。
#if defined(__GNUC__)
#define EXAMPLE_PRINTF_LIKE(format_index, first_arg) \
    __attribute__((format(printf, format_index, first_arg)))
#else
#define EXAMPLE_PRINTF_LIKE(format_index, first_arg)
#endif

namespace example {

// >>> generated example-contract clauses: do not edit by hand, rendered from the operator-contract clause truth
// 以下取值由两语言共用的 operator 契约条款真源渲染而来；手改无效，会被视为“派生副本过期”。
constexpr const char* kDefaultRobotAddress = "192.168.168.168:7447";
constexpr const char* kDefaultNamespace = "robot168";

// 用法尾部的两个位置参数：数组顺序即用法文本顺序，说明行与缺省值一并在此。
struct UsageTailArg {
    const char* token;
    const char* help;
    const char* default_text;
};
constexpr UsageTailArg kUsageTail[] = {
    {"[<ip>:<port>]", "robot-side zenoh bridge address", "192.168.168.168:7447"},
    {"[namespace]", "keyexpr prefix matching the bridge; \"\" when the bridge has none", "robot168"},
};

// 判定前缀词汇：前缀含其后的空格，紧接判定消息。
constexpr const char* kVerdictPass = "[PASS] ";
constexpr const char* kVerdictFail = "[FAIL] ";
constexpr const char* kVerdictWarn = "[WARN] ";
constexpr const char* kVerdictInfo = "[INFO] ";
// <<< generated example-contract clauses

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

// 判定输出与退出码积累：判定前缀取自生成区的词汇表。
// 任一次 Fail() 之后 Failed() 恒为真，Run() 据此返回 1；示例不自己返回失败码。
class Verdicts {
public:
    bool Failed() const { return failed_; }

    void Pass(const char* format, ...) EXAMPLE_PRINTF_LIKE(2, 3);
    void Fail(const char* format, ...) EXAMPLE_PRINTF_LIKE(2, 3);
    void Warn(const char* format, ...) EXAMPLE_PRINTF_LIKE(2, 3);
    void Info(const char* format, ...) EXAMPLE_PRINTF_LIKE(2, 3);

private:
    bool failed_ = false;  // 出现过 [FAIL]：粘住，不因后续判定通过而清零
};

// 四个判定方法只有前缀与"是否粘住失败"不同：取变参 → 前缀 + 一行消息 + 换行（消息
// 按 std::printf 的格式串书写）→ Fail 额外置上失败标记。C++ 不能把变参转发给另一个
// 函数，所以这段形状由这一份定义展开成四个方法；前缀取自生成区。
#define EXAMPLE_VERDICT_METHOD(name, prefix, stick_failure) \
    inline void Verdicts::name(const char* format, ...) {   \
        if (stick_failure) {                                \
            failed_ = true;                                 \
        }                                                   \
        va_list args;                                       \
        va_start(args, format);                             \
        std::printf("%s", prefix);                          \
        std::vprintf(format, args);                         \
        std::printf("\n");                                  \
        va_end(args);                                       \
    }

EXAMPLE_VERDICT_METHOD(Pass, kVerdictPass, false)
EXAMPLE_VERDICT_METHOD(Fail, kVerdictFail, true)
EXAMPLE_VERDICT_METHOD(Warn, kVerdictWarn, false)
EXAMPLE_VERDICT_METHOD(Info, kVerdictInfo, false)

#undef EXAMPLE_VERDICT_METHOD

// 一个可选的位置参数：示例只声明一次，解析与 --help 用法文本都从这份声明派生。
//
// metavar      用法文本里的占位名，例如 "target_mm"
// default_text 不填时使用的文本；与显式取值走同一条 parse 路径，缺省值只写在这里
// help         --help 说明行（英文，与示例的运行时输出一致）
// parse        解析并校验；失败时把这一行的原因写进 error，Run() 打印 [FAIL] 并退出 1
struct PositionalArg {
    const char* metavar;
    const char* default_text;
    const char* help;
    std::function<bool(const char* text, std::string& error)> parse;
};

// 数字位置参数共用的解析：非数字是非法调用，不做静默回退。name 出现在失败原因里
// （"bad <name>: <text> (expected <expected>)"），结果写进 out；与 Python 侧
// parse_double 同形。
inline std::function<bool(const char* text, std::string& error)> ParseDouble(
    const char* name, const char* expected, double& out) {
    return [name, expected, &out](const char* text, std::string& error) {
        char* end = nullptr;
        const double value = std::strtod(text, &end);
        if (end == text || *end != '\0') {
            error = std::string("bad ") + name + ": " + text + " (expected " + expected + ")";
            return false;
        }
        out = value;
        return true;
    };
}

// --help 说明行：占位名对齐后接说明与缺省值。
inline void PrintArgLine(const std::string& token, const char* help, const char* default_text,
                         size_t width) {
    std::printf("%-*s  %s (default %s)\n", static_cast<int>(width), token.c_str(), help,
                default_text);
}

// 用法文本：示例自身的参数按声明顺序在前，尾部两个参数按生成区的顺序在后。
inline void PrintUsage(const std::string& program, const std::vector<PositionalArg>& args) {
    std::printf("usage: %s", program.c_str());
    for (const PositionalArg& arg : args) {
        std::printf(" [%s]", arg.metavar);
    }
    for (const UsageTailArg& tail : kUsageTail) {
        std::printf(" %s", tail.token);
    }
    std::printf("\n\n");
    size_t width = 0;
    for (const UsageTailArg& tail : kUsageTail) {
        const size_t own = std::strlen(tail.token);
        if (own > width) {
            width = own;
        }
    }
    for (const PositionalArg& arg : args) {
        const size_t own = (std::string("[") + arg.metavar + "]").size();
        if (own > width) {
            width = own;
        }
    }
    for (const PositionalArg& arg : args) {
        PrintArgLine(std::string("[") + arg.metavar + "]", arg.help, arg.default_text, width);
    }
    for (const UsageTailArg& tail : kUsageTail) {
        PrintArgLine(tail.token, tail.help, tail.default_text, width);
    }
}

// --help 的用法行只写可执行文件名：Windows 上探到的 argv[0] 带路径与 .exe。
inline std::string ProgramName(const char* argv0) {
    const std::string path = argv0 == nullptr ? "" : argv0;
    const size_t slash = path.find_last_of("/\\");
    std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
    if (name.size() > 4 && name.compare(name.size() - 4, 4, ".exe") == 0) {
        name.resize(name.size() - 4);
    }
    return name;
}

// 示例的契约外壳：--help → 参数解析 → 日志初始化 → 打开会话并构造 Robot → Ready()
// 门禁，就绪后调用 body。body 只写业务动作；判定用 Verdicts，退出码由判定积累。
// 会话是本函数的局部对象：body 返回后先于它析构 Robot，再由最后一个持有者（这里）
// 释放会话，示例的每条退出路径都自动走到这一步。
//
// body 收到 harness 实际使用的配置（示例若要打印自己连的地址就取它），以及契约对象。
inline int Run(int argc, char** argv, const std::vector<PositionalArg>& args,
               const std::function<void(shidou::robot::Robot&, const shidou::comm::ZenohConfig&,
                                        Verdicts&)>& body) {
    const std::string program = ProgramName(argc > 0 ? argv[0] : nullptr);
    // --help / -h 先于一切：不初始化日志、不建会话、不构造 Robot。
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            PrintUsage(program, args);
            return 0;
        }
    }

    Verdicts verdicts;
    // 位置参数：示例自身的参数在前，随后 [<ip>:<port>] 与 [namespace]；多出的即非法调用。
    const int positionals = static_cast<int>(args.size()) + 2;
    if (argc - 1 > positionals) {
        verdicts.Fail("unexpected argument: %s (see --help)", argv[positionals + 1]);
        return 1;
    }
    for (size_t i = 0; i < args.size(); ++i) {
        // 不填时用 default_text：缺省值与显式取值经过同一条解析与校验。
        const char* text =
            static_cast<int>(i) + 1 < argc ? argv[i + 1] : args[i].default_text;
        std::string error;
        if (!args[i].parse(text, error)) {
            verdicts.Fail("%s", error.c_str());
            return 1;
        }
    }
    std::string robot_address = kDefaultRobotAddress;
    std::string ns = kDefaultNamespace;
    if (argc > static_cast<int>(args.size()) + 1) {
        robot_address = argv[args.size() + 1];
    }
    if (argc > static_cast<int>(args.size()) + 2) {
        ns = argv[args.size() + 2];
    }

    shidou::comm::ZenohConfig cfg;
    cfg.robot_address = robot_address;
    cfg.namespace_ = ns;
    shidou::InitLogging("info");

    // 会话在本函数作用域内：打开失败与旧形态的 Robot 构造失败看到同一行 [FAIL]。
    auto session = std::make_shared<shidou::comm::ZenohSession>();
    if (!session->Open(cfg)) {
        verdicts.Fail("comm init failed: %s", session->LastError().c_str());
        return 1;
    }
    shidou::robot::Robot robot(session);
    if (!robot.Ready()) {
        verdicts.Fail("%s", robot.LastError().c_str());
        return 1;
    }
    body(robot, cfg, verdicts);
    return verdicts.Failed() ? 1 : 0;
}

} // namespace example

#undef EXAMPLE_PRINTF_LIKE
